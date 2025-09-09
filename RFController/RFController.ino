#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define SERIAL_BAUD_RATE 115200

#define CE_PIN 9
#define CSN_PIN 10

#define V_REF 3.6
#define ADC_RES_BITS 10.0
#define ADC_STEPS (1 << int(ADC_RES_BITS)) - 1

#define JOYSTICK_VERTICAL_ANALOG_PIN A0
#define JOYSTICK_HORIZONTAL_ANALOG_PIN A1
#define JOYSTICK_SELECT_BUTTON_PIN 2

#define JOYSTICK_LOWER_BOUND -128
#define JOYSTICK_UPPER_BOUND 127
#define JOYSTICK_DRIFT_THRESHOLD 8 // Reading under which we assume the joystick is effectively zero

#define MIN_ACK_REQUEST_TIMEOUT 2
#define MAX_ACK_REQUEST_TIMEOUT 255
#define ACK_REQUEST_TIMEOUT_INCREMENT 10

#define CONTROL_PACKET_ID 'C'
#define ACK_PACKET_ID 'A'

const uint8_t CONTROLLER_ADDRESS[5] = { 0x45, 0x12, 0x78, 0x08, 0xAD };
const uint8_t RECEIVER_ADDRESS[5] = { 0x46, 0x12, 0x78, 0x08, 0xAD };

struct ControlPacket {
  char id = CONTROL_PACKET_ID;
  int8_t vertical;    // Vertical (forward / back) velocity
  int8_t horizontal;  // Horizontal (left / right) velocity
  bool select;        // Whether the joystick select button is pressed

  ControlPacket() {}
  ControlPacket(void *data) {
    char *p0 = (char *)data;
    id = *p0;
    int8_t *p1 = (int8_t *)(p0 + 1);
    vertical = *p1;
    horizontal = *(p1 + 1);
    bool *p2 = (bool *)(p1 + 2);
    select = *p2;
  }

  void *serialize() {
    char *p0 = (char *)malloc(sizeof(ControlPacket));
    *p0 = id;
    int8_t *p1 = (int8_t *)(p0 + 1);
    p1[0] = vertical;
    p1[1] = horizontal;
    bool *p2 = (bool *)(p1 + 2);
    *p2 = select;
    return (void *)p0;
  }

  void print() {
    Serial.print(F("CTRL -> Horizontal: "));
    Serial.print(horizontal);
    Serial.print(F(", Vertical: "));
    Serial.print(vertical);
    Serial.print(F(", Select: "));
    Serial.println(select ? "True" : "False");
  }
};

struct AckPacket {
  char id = ACK_PACKET_ID;
  uint8_t listenTimeoutMillis;  // Number of milliseconds the controller will listen for an ACK

  AckPacket() {}
  AckPacket(void *data) {
    char *p0 = (char *)data;
    id = *p0;
    uint8_t *p1 = (uint8_t*)(data + 1);
    listenTimeoutMillis = *p1;
  }

  void *serialize() {
    char *p0 = (char *)malloc(sizeof(AckPacket));
    *p0 = id;
    uint8_t *p1 = (uint8_t *)(p0 + 1);
    *p1 = listenTimeoutMillis;
    return (void *)p0;
  }
};

uint8_t radioChannel = 69;

RF24 radio(CE_PIN, CSN_PIN, 4000000);

void setup() {
  while (!Serial) {}
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) {}
  printf_begin();

  pinMode(JOYSTICK_SELECT_BUTTON_PIN, INPUT);

  resetRadio();
  radio.printPrettyDetails();
}

void loop() {
  for (uint8_t i = 1; i > 0; i++) {
    sendControlPacket();
  }
  requestAcknowledgement();
}

void sendControlPacket() {
  ControlPacket *packet = new ControlPacket();
  packet->vertical = analogReadJoystick(JOYSTICK_VERTICAL_ANALOG_PIN);
  packet->horizontal = analogReadJoystick(JOYSTICK_HORIZONTAL_ANALOG_PIN);
  packet->select = digitalRead(JOYSTICK_SELECT_BUTTON_PIN) == LOW;
  void *serializedPacket = packet->serialize();
  radio.writeFast(serializedPacket, sizeof(ControlPacket));
  radio.txStandBy(0, true);
  free(serializedPacket);
  delete packet;
}

uint8_t numFailedAckRequests = 0;
void requestAcknowledgement() {
  AckPacket *ackRequestPacket = new AckPacket();
  if ((uint8_t)(numFailedAckRequests + ACK_REQUEST_TIMEOUT_INCREMENT) < numFailedAckRequests) {
    // Integer overflow here means that we have been waiting a long time for an acknowledgement.
    // The receiver may not be there, so we need to light up an indicator on the controller to convey that connection is lost
    // TODO : Implement this
  } else {
    numFailedAckRequests += ACK_REQUEST_TIMEOUT_INCREMENT;
  }
  ackRequestPacket->listenTimeoutMillis = 10;
  radio.stopListening();
  delayMicroseconds(150);

  void *serializedAckRequestPacket = ackRequestPacket->serialize();
  radio.writeFast(serializedAckRequestPacket, sizeof(AckPacket));
  radio.txStandBy(0, true);
  free(serializedAckRequestPacket);

  radio.flush_rx(); // Make sure all the RX buffers are empty before we listen for an ACK
  radio.startListening();

  Serial.print("Requested acknowledgement with timeout of ");
  Serial.print(ackRequestPacket->listenTimeoutMillis);
  Serial.print(F(". "));

  unsigned long startTimeout = millis();
  while (!radio.available()) {
    if (millis() - startTimeout > ackRequestPacket->listenTimeoutMillis) {
      break;
    }
    delayMicroseconds(100);
  }

  delete ackRequestPacket;

  if (!radio.available()) {
    Serial.println(" Acknowledgement not received");
    resetRadio();
    return;
  }

  uint8_t payloadSize = radio.getDynamicPayloadSize();

  if (payloadSize != sizeof(AckPacket)) {
    Serial.print("Malformed ack response: ");
    Serial.println(payloadSize);
    radio.flush_rx();
    radio.stopListening();
    return;
  }

  void *data = (void *)malloc(payloadSize);
  radio.read(data, payloadSize);
  Serial.print(F("Ack response: "));
  AckPacket *ackResponsePacket = new AckPacket(data);
  if (ackResponsePacket->listenTimeoutMillis == numFailedAckRequests) {
    numFailedAckRequests = 0;
    Serial.println("Acknowledgement successful!");
  } else {
    Serial.print("Received invalid ack response. Expected ");
    Serial.print(numFailedAckRequests);
    Serial.print(" Received: ");
    Serial.println(ackResponsePacket->listenTimeoutMillis);
  }
  radio.stopListening();
  delete ackResponsePacket;
  free(data);
}

void verifyRadio() {
  while (radio.failureDetected || !radio.isChipConnected()) {
    resetRadio();
  }
}

void resetRadio() {
  // initialize the transceiver on the SPI bus
  while (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!"));
    delay(10);
  }
  radio.failureDetected = 0;
  radio.txDelay = 0;
  radio.csDelay = 0;

  radio.setPALevel(RF24_PA_LOW);
  radio.setCRCLength(RF24_CRC_16);
  radio.setChannel(radioChannel);
  radio.setAutoAck(false);

  radio.enableDynamicPayloads();
  radio.enableDynamicAck();

  radio.stopListening(RECEIVER_ADDRESS);
  radio.openReadingPipe(1, CONTROLLER_ADDRESS);

  delay(1);
}

int8_t analogReadJoystick(int pin) {
  int8_t value = (int8_t)map(analogRead(pin), 0, ADC_STEPS, JOYSTICK_LOWER_BOUND, JOYSTICK_UPPER_BOUND);
  if (abs(value) < JOYSTICK_DRIFT_THRESHOLD) {
    return 0;
  }
  return value;
}
