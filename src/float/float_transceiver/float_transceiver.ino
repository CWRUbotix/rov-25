// FLOAT TRANSCEIVER sketch originally built from:
//  rf95_client.pde
//  -*- mode: C++ -*-
//
// REQUIRED LIBRARIES:
// RadioHead v1.122.1 by Mike McCauley
// Blue Robotics MS5837 Library v1.1.1 by BlueRobotics
// PicoEncoder v1.0.9 by Paulo Marques

#include <RH_RF95.h>
#include <SPI.h>
#include "PicoEncoder.h"

#include "MS5837.h"
#include "rov_common.hpp"

// H-bridge direction control pins
const uint8_t MOTOR_PWM = 6;  // Leave 100% cycle for top speed
const uint8_t PUMP_PIN = 9;   // Set high for pump (CW when facing down)
const uint8_t SUCK_PIN = 10;  // Set high for suck (CCW when facing down)

// Limit switch pins
const uint8_t LIMIT_FULL = 12;   // Low when syringe is full
const uint8_t LIMIT_EMPTY = 11;  // Low when syringe is empty

// Encoder initialization
PicoEncoder encoder;
const uint8_t ENCODER_PIN = 4;

const uint8_t TEAM_NUM = 25;
const uint32_t PACKET_SEND_INTERVAL = 1000;
const uint32_t FLOAT_PKT_RX_TIMEOUT = 900;
const uint8_t JUDGE_PKT_SIZE = 30;

#ifdef DO_DEBUGGING
const uint32_t PRESSURE_READ_INTERVAL = 200;
const uint32_t PROFILE_SEGMENT = 10000;
#else
const uint32_t PRESSURE_READ_INTERVAL = 5000;
const uint32_t PROFILE_SEGMENT = 60000;
#endif

const uint32_t ONE_MINUTE = 60000;

// Schedule (all delays in ms)
const uint32_t RELEASE_MAX = 8 * ONE_MINUTE;
const uint32_t SUCK_MAX = PROFILE_SEGMENT;
const uint32_t DESCEND_TIME = PROFILE_SEGMENT;
const uint32_t PUMP_MAX = PROFILE_SEGMENT;
const uint32_t ASCEND_TIME = 0;
const uint32_t TX_MAX_TIME = 2 * ONE_MINUTE;

uint32_t stageStartTime;
uint32_t previousPressureReadTime;
uint32_t previousPacketSendTime;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Singleton instance of the pressure sensor driver
MS5837 pressureSensor;

byte packets[2][PKT_LEN];
int packetIndex = PKT_HEADER_LEN;

enum class StageType { Deploy, WaitDeploying, WaitTransmitting, 
  Suck, Pump, Descending, Ascending, HoldDepth};
enum class OverrideState { NoOverride, Stop, Suck, Pump };
enum class MotorState { Stop, Suck, Pump, Error };

StageType stage = StageType::Deploy;
OverrideState overrideState = OverrideState::NoOverride;
MotorState motorState = MotorState::Error;

uint8_t profileNum = 0;
uint8_t profileHalf = 0;

void setup() {
  Serial.begin(115200);
  // Wait until serial console is open; remove if not tethered to computer
  while (!Serial){}

  Serial.println("Float Transceiver");
  Serial.println();

  stageStartTime = millis();
  previousPressureReadTime = millis();
  previousPacketSendTime = millis();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(LIMIT_EMPTY, INPUT_PULLUP);
  pinMode(LIMIT_FULL, INPUT_PULLUP);

  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(SUCK_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);

  digitalWrite(MOTOR_PWM, LOW);
  digitalWrite(SUCK_PIN, LOW);
  digitalWrite(PUMP_PIN, LOW);

  clearPacketPayloads();

  packets[0][PKT_IDX_TEAM_NUM] = TEAM_NUM;
  packets[0][PKT_IDX_PROFILE_HALF] = 0;

  packets[1][PKT_IDX_TEAM_NUM] = TEAM_NUM;
  packets[1][PKT_IDX_PROFILE_HALF] = 1;

  initRadio();
  initPressureSensor();
  encoder.begin(ENCODER_PIN);
}

void loop(){
  while(overrideState != OverrideState::NoOverride){
    loopOverride();
  }

  /* Each stage is structured as follows:
  *
  * case [stage name] (entry point)
  *   [code for stage]
  * while (![override or exit condition])
  *   [check for overrides or exit conditions]
  * if ([override])
  *   break; (and go to override)
  * stage = [next stage]
  */
  switch (stage) {
  case StageType::Deploy:
    // Deploy code
    // Await override or exit condition
    while(overrideState == OverrideState::NoOverride){}
    if(overrideState != OverrideState::NoOverride) break;
    stage = StageType::WaitDeploying;
  case StageType::WaitDeploying:
    // WaitDeploying code
    // Await override or exit condition
    while(overrideState == OverrideState::NoOverride){}
    if(overrideState != OverrideState::NoOverride) break;
    stage = StageType::Suck;
  case StageType::Suck:
    // Suck code
    // Await override or exit condition
    while(overrideState == OverrideState::NoOverride){}
    if(overrideState != OverrideState::NoOverride) break;
    stage = StageType::Descending;
  case StageType::Descending:
    // Descending code
    // Await override or exit condition
    while(overrideState == OverrideState::NoOverride){}
    if(overrideState != OverrideState::NoOverride) break;
    stage = StageType::HoldDepth;
  case StageType::HoldDepth:
    // HoldDepth code
    // Await override or exit condition
    while(overrideState == OverrideState::NoOverride){}
    if(overrideState != OverrideState::NoOverride) break;
    stage = StageType::Pump;
  case StageType::Pump:
    // Pump code
    // Await override or exit condition
    while(overrideState == OverrideState::NoOverride){}
    if(overrideState != OverrideState::NoOverride) break;
    stage = StageType::Ascending;
  case StageType::Ascending:
    // Ascending code
    // Await override or exit condition
    while(overrideState == OverrideState::NoOverride){}
    if(overrideState != OverrideState::NoOverride) break;
    stage = StageType::WaitTransmitting;
  case StageType::WaitTransmitting:
    // WaitTransmitting code
    // Await override or exit condition
    while(overrideState == OverrideState::NoOverride){}
    if(overrideState != OverrideState::NoOverride) break;
    stage = StageType::Descending;
  break;
  default:
    Serial.println("Error: Undefined float stage");
    break;
  }
}

void loopOverride(){
  if (overrideState == OverrideState::Suck) {
    if (digitalRead(LIMIT_FULL) == HIGH) {
      suck();
    }
    else {
      stop();
      overrideState = OverrideState::Stop;
    }
  }
  else if (overrideState == OverrideState::Pump) {
    if (digitalRead(LIMIT_EMPTY) == HIGH) {
      pump();
    }
    else {
      stop();
      overrideState = OverrideState::Stop;
    }
  }
  else if (overrideState == OverrideState::Stop) {
    stop();
  }
}

/******* Control Methods *******/

void pump() {
  Serial.println("PUMPING");
  digitalWrite(MOTOR_PWM, HIGH);
  digitalWrite(PUMP_PIN, HIGH);
  digitalWrite(SUCK_PIN, LOW);
  motorState = MotorState::Pump;
}

void suck() {
  Serial.println("SUCKING");
  digitalWrite(MOTOR_PWM, HIGH);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(SUCK_PIN, HIGH);
  motorState = MotorState::Suck;
}

void stop() {
  Serial.println("STOPPING");
  digitalWrite(MOTOR_PWM, LOW);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(SUCK_PIN, LOW);
  motorState = MotorState::Stop;
}

bool receiveCommand() {
  Serial.println("Receiving command...");

  if (!rf95.waitAvailableTimeout(FLOAT_PKT_RX_TIMEOUT)) {
    return false;
  }

  Serial.println("RF has signal");
  byte len = RH_RF95_MAX_MESSAGE_LEN;
  byte byteBuffer[len];

  if (!rf95.recv(byteBuffer, &len)) {
    Serial.println("Receive failed");
    return false;
  }
  if (!len) {
    Serial.println("Received with length 0; dropping");
    return false;
  }

  byteBuffer[len] = 0;
  char* charBuf = reinterpret_cast<char*>(byteBuffer);

  serialPrintf("Received [%d]: '%s'\n", len, charBuf);

  String response;
  bool shouldSubmerge = false;

  if (strcmp(charBuf, "submerge") == 0) {
    response = "ACK SUBMERGING";
    shouldSubmerge = true;
  }
  else if (strcmp(charBuf, "pump") == 0) {
    if (motorState != MotorState::Suck) {
      response = "ACK PUMPING";
      overrideState = OverrideState::Pump;
    }
    else {
      response = "NACK RETURN FIRST";
    }
  }
  else if (strcmp(charBuf, "suck") == 0) {
    if (motorState != MotorState::Pump) {
      response = "ACK SUCKING";
      overrideState = OverrideState::Suck;
    }
    else {
      response = "NACK RETURN FIRST";
    }
  }
  else if (strcmp(charBuf, "stop") == 0) {
    response = "ACK STOPPING";
    overrideState = OverrideState::Stop;
  }
  else if (strcmp(charBuf, "return") == 0) {
    response = "ACK RETURNING TO SCHEDULE";
    stop();
    overrideState = OverrideState::NoOverride;
  }
  else {
    response = "NACK INVALID COMMAND";
  }

  byte responseBytes[response.length() + 1];
  response.getBytes(responseBytes, response.length() + 1);
  rf95.send(responseBytes, response.length() + 1);
  rf95.waitPacketSent();
  return shouldSubmerge;
}

void transmitPressurePacket() {
  for (int half = 0; half < 2; half++) {
    serialPrintf("Sending packet #%d half %d with content {", profileNum, half);
    for (int p = 0; p < PKT_LEN; p++) {
      serialPrintf("%d, ", packets[half][p]);
    }
    Serial.println("}");

    packets[half][PKT_IDX_PROFILE_NUM] = profileNum;

    rf95.send(packets[half], PKT_LEN);
    rf95.waitPacketSent();
  }
}

void clearPacketPayloads() {
  for (int half = 0; half < 2; half++) {
    for (int i = PKT_HEADER_LEN; i < PKT_LEN; i++) {
      packets[half][i] = 0;
    }
  }
}

/******* Setup Methods *******/

void initRadio() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println("Feather RFM95 TX Test!");
  Serial.println();

  // Manually reset radio module
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("RFM95 radio init failed");
    while (1) {}
  }
  Serial.println("RFM95 radio init OK!");

  // Defaults after init are: 434.0MHz, modulation GFSK_Rb250Fd250
  // +13dbM (for low power module), no encryption
  // But we override frequency
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  serialPrintf("RFM95 radio @%d MHz\n", static_cast<int>(RF95_FREQ));
}

void initPressureSensor() {
  pressureSensor.init();

  pressureSensor.setModel(MS5837::MS5837_02BA);
  pressureSensor.setFluidDensity(997);  // kg/m^3

  serialPrintf("PRESSURE: %f\n", pressureSensor.pressure());
}
