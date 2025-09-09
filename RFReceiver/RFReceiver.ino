#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define SERIAL_BAUD_RATE 115200

#define CE_PIN 9
#define CSN_PIN 10

#define DRIVE_MOTOR_PIN_1 4
#define DRIVE_MOTOR_PIN_2 5
#define STEERING_MOTOR_PIN_1 6
#define STEERING_MOTOR_PIN_2 7
#define H_BRIDGE_INTERRUPT_PIN 2

#define JOYSTICK_LOWER_BOUND -128
#define JOYSTICK_UPPER_BOUND 127

#define PWM_POSITION_FACTOR 1
#define PWM_MAX_ABS_POSITION 100 * PWM_POSITION_FACTOR

#define PWM_MAX_DUTY_CYCLE 0.75
#define PWM_MIN_DUTY_CYCLE 0.10
#define PWM_DUTY_CYCLE_DELTA_THRESHOLD 0.025

#define COASTING_TIMEOUT_MICROS 100000

#define CONTROL_PACKET_ID 'C'
#define ACK_PACKET_ID 'A'

#define MAX_ALLOWED_PAYLOAD_SIZE 8

const uint8_t CONTROLLER_ADDRESS[5] = { 0x45, 0x12, 0x78, 0x08, 0xAD };
const uint8_t RECEIVER_ADDRESS[5] = { 0x46, 0x12, 0x78, 0x08, 0xAD };

/**
 * ----------------------------------------------
 * --------------- STRUCT DEFS ------------------
 * ----------------------------------------------
 */
struct ControlPacket {
  char id = CONTROL_PACKET_ID;
  int8_t vertical = 0;    // Vertical (forward / back) velocity
  int8_t horizontal = 0;  // Horizontal (left / right) velocity
  bool select = false;        // Whether the joystick select button is pressed

  ControlPacket() {}
  ControlPacket(void *data) {
    apply(data);
  }

  void apply(void *data) {
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
  uint8_t listenTimeoutMillis = 0;  // Number of milliseconds the controller will listen for an ACK

  AckPacket() {}
  AckPacket(void *data) {
    apply(data);
  }
  void apply(void *data) {
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
  float minDutyCycle = PWM_MIN_DUTY_CYCLE;
  float maxDutyCycle = PWM_MAX_DUTY_CYCLE;
  float deltaThreshold = PWM_DUTY_CYCLE_DELTA_THRESHOLD;
  uint64_t previousHighTick = 0;
  uint16_t pulseDivisionMicros = 2000; // 500hz

  PWM(uint8_t p) {
    pin = p;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  void activate() {
    isActive = true;
    pulse();
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
      digitalWrite(pin, !isHigh);
      if (isHigh) {
        previousHighTick = micros();
      }
    }
  }

  void setDutyCycleFromAnalogValue(int8_t analogPinValue) {
    setDutyCycle((float)pow(analogPinValue, 2.0) / (pow(JOYSTICK_UPPER_BOUND, 2.0) / maxDutyCycle));
  }

  void setDutyCycle(float ds) {
    if (ds > maxDutyCycle) {
      ds = maxDutyCycle;
    } else if (ds < minDutyCycle && ds > 0.0) {
      ds = minDutyCycle;
    }
    if (ds + deltaThreshold >= dutyCycle && ds - deltaThreshold <= dutyCycle) {
      return;
    }
    Serial.print(pin);
    Serial.print(F("-> Setting duty cycle : "));
    Serial.println(ds, 4);
    dutyCycle = ds;
  }

  void pulse() {
    if (!isActive || dutyCycle < minDutyCycle) {
      setState(false);
      return;
    }
    
    // Wait until it is time to switch states
    uint32_t currentPulseLength = (uint32_t)((float)pulseDivisionMicros * (isHigh ? dutyCycle : (1.0 - dutyCycle)));
    while (micros() - previousHighTick < currentPulseLength) {}
    setState(!isHigh);
  }
};

struct Motor {
  PWM *pwm1;
  PWM *pwm2;
  bool trackPosition = false;
  int8_t velocity = 0;
  int16_t position = 0;
  int16_t nextPosition = 0;

  Motor(uint8_t pin1, uint8_t pin2) {
    init(pin1, pin2);
  }

  Motor(uint8_t pin1, uint8_t pin2, bool tp) {
    init(pin1, pin2);
    trackPosition = tp;
  }

  void init(uint8_t pin1, uint8_t pin2) {
    pwm1 = new PWM(pin1);
    pwm2 = new PWM(pin2);
  }

  void setPositionFromAnalogValue(int8_t p) {
    setPosition(PWM_POSITION_FACTOR * p);
  }

  void setPosition(int16_t p) {
    if (abs(p) > PWM_MAX_ABS_POSITION) {
      p = PWM_MAX_ABS_POSITION * (p > 0 ? 1 : -1);
    }
    if (nextPosition != p) {
      nextPosition = p;
      //Serial.print("Next Position: ");
      //Serial.println(nextPosition);
    }
  }

  void setVelocity(int8_t v) {
    velocity = v;
  }

  void stop() {
    pwm1->deactivate();
    pwm2->deactivate();
  }

  void pulse() {
    //currentControllerState->print();
    if (trackPosition) {
      if (velocity > 0) {
        position++;
      } else if (velocity < 0) {
        position--;
      }
      if (position > nextPosition) {
        velocity = -127;
      } else if (position < nextPosition) {
        velocity = 127;
      } else {
        velocity = 0;
      }
    }
    if (velocity > 0) {
      // Forward drive
      pwm1->deactivate(HIGH);
      pwm2->setDutyCycleFromAnalogValue(velocity);
      pwm2->activate();
    } else if (velocity < 0) {
      // Reverse drive
      pwm2->deactivate(HIGH);
      pwm1->setDutyCycleFromAnalogValue(abs(velocity));
      pwm1->activate();
    } else {
      // Stop
      stop();
    }
  }
};

// TODO : Implement channel change request handling
uint8_t radioChannel = 69;

// Global static variales
RF24 *radio = new RF24(CE_PIN, CSN_PIN, 4000000);
Motor *driveMotor = new Motor(DRIVE_MOTOR_PIN_1, DRIVE_MOTOR_PIN_2);
//Motor *steeringMotor = new Motor(STEERING_MOTOR_PIN_1, STEERING_MOTOR_PIN_2, true);
PWM *steeringMotor = new PWM(STEERING_MOTOR_PIN_1);

/**
 * ----------------------------------------------
 * -------------- SETUP & LOOP ------------------
 * ----------------------------------------------
 */
void setup() {
  while (!Serial) {}
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) {}
  printf_begin();

  steeringMotor->pulseDivisionMicros = 20000;
  steeringMotor->minDutyCycle = 0.75 / 20.0;
  steeringMotor->maxDutyCycle = 2.25 / 20.0;
  steeringMotor->deltaThreshold = 0.001;
  steeringMotor->setDutyCycle(1.5/20.0);
  steeringMotor->activate();

  attachInterrupt(digitalPinToInterrupt(H_BRIDGE_INTERRUPT_PIN), handleHBridgeFailure, FALLING);

  resetRadio();
  radio->printPrettyDetails();
}

void loop() {
  ControlPacket *controllerState = readControllerState();
  if (controllerState != NULL) {
    //controllerState->print();
    driveMotor->setVelocity(controllerState->vertical);
    delete controllerState;
    //steeringMotor->setPositionFromAnalogValue(controllerState->horizontal);
    float ds = (float)map(controllerState->horizontal, JOYSTICK_LOWER_BOUND, JOYSTICK_UPPER_BOUND, 75, 225) / (100.0 * 20.0);
    steeringMotor->setDutyCycle(ds);
  }
  driveMotor->pulse();
  steeringMotor->pulse();
}

void handleHBridgeFailure() {
  // Stop the motors on failure
  // This usually happens when you change directions too fast
  driveMotor->stop();
}

uint8_t unavailableCount = 0;
ControlPacket *readControllerState() {
  if (!radio->available()) {
    if (++unavailableCount == 0) {
      resetRadio();
    } else {
      delayMicroseconds(100);
    }
    return NULL;
  }
  unavailableCount = 0;

  uint8_t payloadSize = radio->getDynamicPayloadSize();  // get the size of the payload
  if (payloadSize == 0 || payloadSize > MAX_ALLOWED_PAYLOAD_SIZE) {
    radio->flush_rx();
    return NULL;
  }
  void *data = (void *)malloc(payloadSize);
  realloc(data, payloadSize);
  radio->read(data, payloadSize);
  char id = *((char *)data);
  if (CONTROL_PACKET_ID == id) {
    ControlPacket *packet = new ControlPacket(data);
    free(data);
    return packet;
  }
  if (ACK_PACKET_ID == id) {
    handleAckRequest(data);
  } else {
    Serial.print(F("ERROR: Unexpected payload: "));
    uint8_t *di = (uint8_t *)data;
    for (uint8_t i = 0; i < payloadSize; i++) {
      Serial.print(di[i]);
      Serial.print(F(" "));
    }
    Serial.println(F(""));
  }
  free(data);
  return NULL;
}

void handleAckRequest(void *data) {
  AckPacket *packet = new AckPacket(data);
  const unsigned long startTime = millis();
  char *serializedResponsePacket = packet->serialize();
  while (millis() - startTime < packet->listenTimeoutMillis) {
    radio->stopListening();
    Serial.println("Writing ACK");
    radio->writeFast(serializedResponsePacket, sizeof(AckPacket));
    radio->txStandBy(0, true);
    radio->startListening();

    driveMotor->pulse();
    steeringMotor->pulse();

    // If a packet is available, stop sending ACKs and handle the packet
    if (radio->available() && radio->getDynamicPayloadSize() > 0) {
      break;
    }
  }
  free(serializedResponsePacket);
  delete packet;
}

void resetRadio() {
  // initialize the transceiver on the SPI bus
  while (!radio->begin()) {
    Serial.println(F("radio hardware is not responding!"));
    delay(10);
  }
  radio->failureDetected = 0;
  radio->txDelay = 0;
  radio->csDelay = 0;

  radio->setPALevel(RF24_PA_LOW);
  radio->setCRCLength(RF24_CRC_16);
  radio->setChannel(radioChannel);
  radio->setAutoAck(false);

  radio->enableDynamicPayloads();
  radio->enableDynamicAck();

  radio->stopListening(CONTROLLER_ADDRESS);
  radio->openReadingPipe(1, RECEIVER_ADDRESS);
  radio->startListening();

  delayMicroseconds(150);
}