#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define SERIAL_BAUD_RATE 115200

#define CE_PIN 9
#define CSN_PIN 10

#define DRIVE_MOTOR_PIN_1 4
#define DRIVE_MOTOR_PIN_2 5
#define HBRIDGE_INTERRUPT_PIN 2

#define JOYSTICK_LOWER_BOUND -128
#define JOYSTICK_UPPER_BOUND 127

#define PWM_MAX_DUTY_CYCLE 0.75
#define PWM_MIN_DUTY_CYCLE 0.075
#define PWM_DUTY_CYCLE_DELTA_THRESHOLD 0.025

#define COASTING_TIMEOUT_MICROS 100000

#define CONTROL_PACKET_ID 'C'
#define ACK_PACKET_ID 'A'

#define MAX_ALLOWED_PAYLOAD_SIZE 8

const uint8_t CONTROLLER_ADDRESS[5] = { 0x45, 0x12, 0x78, 0x08, 0xAD };
const uint8_t RECEIVER_ADDRESS[5] = { 0x46, 0x12, 0x78, 0x08, 0xAD };

struct ControlPacket {
  char id = CONTROL_PACKET_ID;
  int8_t vertical = 0;    // Vertical (forward / back) velocity
  int8_t horizontal = 0;  // Horizontal (left / right) velocity
  bool select = false;        // Whether the joystick select button is pressed

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
ControlPacket *previousControllerState;
ControlPacket *currentControllerState;

struct AckPacket {
  char id = ACK_PACKET_ID;
  uint8_t listenTimeoutMillis = 0;  // Number of milliseconds the controller will listen for an ACK

  AckPacket() {}
  AckPacket(void *data) {
    char *p0 = (char *)data;
    id = *p0;
    uint8_t *p1 = (uint8_t *)(data + 1);
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

struct PWM {
  uint8_t pin;
  bool isActive = false;
  bool isHigh = false;
  float dutyCycle = 0.0;
  float nextDutyCycle = 0.0;
  uint64_t previousHighTick = 0;
  uint16_t pulseDivisionMicros = 2000; // 200hz

  PWM(uint8_t p) {
    pin = p;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  void activate() {
    isActive = true;
    ping();
  }

  void deactivate() {
    deactivate(false);
  }

  void deactivate(bool staticState) {
    isActive = false;
    setState(staticState);
  }

  void setState(bool state) {
    if (isHigh != state) {
      isHigh = state;
      digitalWrite(pin, isHigh ? HIGH : LOW);
      if (isHigh) {
        previousHighTick = micros();
      }
    }
  }

  void setDutyCycleFromAnalogValue(int8_t analogPinValue) {
    setDutyCycle((float)pow(analogPinValue, 2.0) / (pow(JOYSTICK_UPPER_BOUND, 2.0) / PWM_MAX_DUTY_CYCLE));
  }

  void setDutyCycle(float ds) {
    if (ds > PWM_MAX_DUTY_CYCLE) {
      ds = PWM_MAX_DUTY_CYCLE;
    } else if (ds < PWM_MIN_DUTY_CYCLE && ds > 0.0) {
      ds = PWM_MIN_DUTY_CYCLE;
    }
    if (ds + PWM_DUTY_CYCLE_DELTA_THRESHOLD >= dutyCycle && ds - PWM_DUTY_CYCLE_DELTA_THRESHOLD <= dutyCycle) {
      return;
    }
    dutyCycle = ds;
    Serial.print(pin);
    Serial.print(F("-> Setting duty cycle : "));
    Serial.println(dutyCycle, 4);
  }

  void ping() {
    if (!isActive || dutyCycle < PWM_MIN_DUTY_CYCLE) {
      setState(false);
      return;
    }
    
    // Wait until it is time to drop to low state
    uint32_t currentPulseLength = (uint32_t)((float)pulseDivisionMicros * (isHigh ? dutyCycle : (1.0 - dutyCycle)));
    uint32_t timeToSleep = micros() - previousHighTick + currentPulseLength;
    delayMicroseconds(timeToSleep);
    setState(!isHigh);
  }
};
PWM *driveMotorPWM1;
PWM *driveMotorPWM2;

// TODO : Implement channel change request handling
uint8_t radioChannel = 69;

RF24 radio(CE_PIN, CSN_PIN, 4000000);

void setup() {
  while (!Serial) {}
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) {}
  printf_begin();

  driveMotorPWM1 = new PWM(DRIVE_MOTOR_PIN_1);
  driveMotorPWM2 = new PWM(DRIVE_MOTOR_PIN_2);
  previousControllerState = new ControlPacket();
  currentControllerState = new ControlPacket();

  attachInterrupt(digitalPinToInterrupt(HBRIDGE_INTERRUPT_PIN), handleHBridgeFailure, FALLING);

  resetRadio();
  radio.printPrettyDetails();
}

void loop() {
  operateDriveMotor();
  checkRadioSignal();
}

void operateDriveMotor() {
  //currentControllerState->print();
  if (currentControllerState->vertical > 0) {
    // Forward drive
    driveMotorPWM1->deactivate();
    driveMotorPWM2->setDutyCycleFromAnalogValue(currentControllerState->vertical);
    driveMotorPWM2->activate();
  } else if (currentControllerState->vertical < 0) {
    // Reverse drive
    driveMotorPWM2->deactivate();
    driveMotorPWM1->setDutyCycleFromAnalogValue(abs(currentControllerState->vertical));
    driveMotorPWM1->activate();
  } else {
    // Stop
    driveMotorPWM1->deactivate();
    driveMotorPWM2->deactivate();
  }
}

void handleHBridgeFailure() {
  // Stop the motors on failure
  // This usually happens when you change directions too fast
  driveMotorPWM1->deactivate();
  driveMotorPWM2->deactivate();
}

uint8_t unavailableCount = 0;
void checkRadioSignal() {
  if (!radio.available()) {
    if (++unavailableCount == 0) {
      resetRadio();
    } else {
      delayMicroseconds(100);
    }
    return;
  }
  unavailableCount = 0;

  uint8_t payloadSize = radio.getDynamicPayloadSize();  // get the size of the payload
  if (payloadSize == 0 || payloadSize > MAX_ALLOWED_PAYLOAD_SIZE) {
    radio.flush_rx();
    return;
  }
  void *data = (void *)malloc(payloadSize);
  realloc(data, payloadSize);
  radio.read(data, payloadSize);
  char id = *((char *)data);
  if (id == ACK_PACKET_ID) {
    handleAckRequest(data);
  } else if (id == CONTROL_PACKET_ID) {
    delete previousControllerState;
    previousControllerState = currentControllerState;
    currentControllerState = new ControlPacket(data);
  } else {
    Serial.print(F("ERROR: Unexpected payload: "));
    uint8_t *di = (uint8_t *)data;
    for (uint8_t i = 0; i < payloadSize; i++) {
      Serial.print(di[i]);
      Serial.print(F(" "));
    }
    Serial.println(F(""));
    delete previousControllerState;
    previousControllerState = currentControllerState;
    currentControllerState = new ControlPacket();
  }
  free(data);
}

void handleAckRequest(void *data) {
  AckPacket *packet = new AckPacket(data);
  /*
  Serial.print("Received ACK Request for ");
  Serial.print(packet->listenTimeoutMillis);
  Serial.print("ms. ");
  */
  const unsigned long startTime = millis();
  char *serializedResponsePacket = packet->serialize();
  while (millis() - startTime < packet->listenTimeoutMillis) {
    operateDriveMotor();

    radio.stopListening();
    delayMicroseconds(150);
    radio.writeFast(serializedResponsePacket, sizeof(AckPacket));
    radio.txStandBy(0, true);
    radio.startListening();

    operateDriveMotor();
    /*
    Serial.print(" ACK Written ");
    Serial.print(sizeof(AckPacket));
    */

    // If a packet is available, stop sending ACKs and handle the packet
    if (radio.available() && radio.getDynamicPayloadSize() > 0) {
      break;
    }
  }
  free(serializedResponsePacket);
  delete (packet);
  //Serial.println(F(""));
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

  radio.stopListening(CONTROLLER_ADDRESS);
  radio.openReadingPipe(1, RECEIVER_ADDRESS);
  radio.startListening();

  delayMicroseconds(150);
}