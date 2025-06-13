// FLOAT TRANSCEIVER
// Originally built from rf95_client.pde
//
// REQUIRED LIBRARIES:
// RadioHead v1.122.1 by Mike McCauley
// Blue Robotics MS5837 Library v1.1.1 by BlueRobotics
// PicoEncoder v1.1.1 by Paulo Marques
// TaskScheduler v3.8.5 by Anatoli Arkhipenko

#include <RHSoftwareSPI.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <TaskScheduler.h>
#include <Wire.h>

#include "MS5837.h"
#include "PicoEncoder.h"
#include "rov_common.hpp"

// H-bridge direction control pins
#define PIN_MOTOR_PUMP 20
#define PIN_MOTOR_SUCK 21

// Limit switch pins
#define PIN_LIMIT_NO 11
#define PIN_LIMIT_NC 10

// Status LED pins
#define PIN_LED_RED   9
#define PIN_LED_BLUE  6
#define PIN_LED_GREEN 12

// The encoder requires two adjacent pins on the PIO
#define PIN_ENCODER 4

const uint8_t TEAM_NUM = 25;
const uint32_t PACKET_SEND_INTERVAL = 1000;
const uint32_t FLOAT_PKT_RX_TIMEOUT = 10;
const uint8_t JUDGE_PKT_SIZE = 30;

// #define DO_DEBUGGING
#ifdef DO_DEBUGGING
const uint32_t PRESSURE_READ_INTERVAL = 200;
const uint32_t PROFILE_SEGMENT = 10000;
#else
const uint32_t PRESSURE_READ_INTERVAL = 5000;
const uint32_t PROFILE_SEGMENT = 60000;
#endif

const uint32_t PRESSURE_TRANSMIT_INTERVAL = 1000;

const uint32_t ONE_MINUTE = 60000;

// Schedule timings (all delays in ms)
const uint32_t AFTER_HOME_WAIT = 10'000;
const uint32_t RELEASE_MAX = 8 * ONE_MINUTE;
const uint32_t SUCK_MAX = PROFILE_SEGMENT;
const uint32_t DESCEND_TIME = 2 * ONE_MINUTE;
const uint32_t PUMP_MAX = PROFILE_SEGMENT;
const uint32_t ASCEND_TIME = ONE_MINUTE;
const uint32_t TX_MAX_TIME = 2 * ONE_MINUTE;
const uint32_t HOLD_TIME = 0;

// Distance from the pressure sensor to the bottom of the float (m)
const float MBAR_TO_METER_OF_HEAD = 0.010199773339984;
const float PRESSURE_SENSOR_VERTICAL_OFFSET = 0;  // 0.62 to measure to bottom of float;
const int AVERAGE_PRESSURE_LEN = 10;

float surfacePressures[AVERAGE_PRESSURE_LEN];
float surfaceAverage = 0;
int surfacePressureIndex = 0;

RHSoftwareSPI softwareSPI;

PicoEncoder encoder;
const long COUNTS_PER_REV =
  64 * 100 * 64;  // Encoder advertises 64 CPR, 100:1 gearbox, PicoEncoder lib multiplies by 64;
const long EMPTY_ANGLE = 232;
const long NEUTRAL_BOUYANCY_ANGLE = 116;

RH_RF95 rf95(RFM95_CS, RFM95_INT, softwareSPI);
MS5837 pressureSensor;

byte packets[2][PKT_LEN];
int packetIndex = PKT_HEADER_LEN;

enum class StageType {
  DeploySuck,
  DeployWait,
  DeployPump,
  WaitDeploying,
  Descending,
  Pump,
  Ascending,
  WaitTransmitting
};

enum class OverrideState { NoOverride, Stop, Suck, Pump };
enum class MotorState { Stop, Suck, Pump, SpeedControl, Error };

StageType stage = StageType::DeploySuck;
OverrideState overrideState = OverrideState::NoOverride;
MotorState motorState = MotorState::Error;

uint8_t profileNum = 0;
uint8_t profileHalf = 0;

uint32_t stageTimeout = 0;

// Colors for status LED
struct COLOR {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

// The number of blinks of the status LED for each error code
enum ERROR_CODES {
  RADIO_INIT = 2,
  ENCODER_INIT = 3,
  PRESSURE_SENSOR_INIT = 4,
  LIMIT_SWITCH_INIT = 5
};

COLOR COLOR_OFF = {0, 0, 0};
COLOR COLOR_ERROR = {255, 0, 0};
COLOR COLOR_INITIALIZING = {255, 255, 0};
COLOR COLOR_READY = {0, 255, 0};
COLOR COLOR_DESCENDING = {0, 0, 255};
COLOR COLOR_ASCENDING = {255, 0, 255};

// Control loop variables
float depthMean;
float velMean;
float depthVar;
float velVar;
float depthVelCovar;
uint32_t lastKalmanUpdate;

const float DEPTH_SENSOR_VAR = 5e-5;  // std dev of 3mm
const float VEL_PROCESS_NOISE = 1e-3;

const float TARGET_DEPTH = 2.5;     // meters
const float DEPTH_TOLERANCE = 0.5;  // 0.5 m in either direction of target, i.e. 2m to 3m

const float DEPTH_P = 15;
const float DEPTH_I = 2;
const float DEPTH_D = 100;

const float MOT_P = 1;

const float DEPTH_ACC_MAX = 7.5;
float depthErrAcc = 0;

// Task scheduler and task declarations
Scheduler taskScheduler;

// Task callback function declarations
void updateEncoderCallback();
void transmitSurfacePressureCallback();
void recordPressureCallback();
void transmitPacketsCallback();

// Task objects
Task taskUpdateEncoder(10, TASK_FOREVER, &updateEncoderCallback);
Task taskTransmitSurfacePressure(
  PRESSURE_TRANSMIT_INTERVAL, TASK_FOREVER, &transmitSurfacePressureCallback);
Task taskRecordPressure(PRESSURE_READ_INTERVAL, TASK_FOREVER, &recordPressureCallback);
Task taskTransmitPackets(PRESSURE_TRANSMIT_INTERVAL, TASK_FOREVER, &transmitPacketsCallback);

void setup() {
  delay(2000);
  Serial.begin(115200);
  Serial.println("Float Transceiver with Task Scheduler");
  Serial.println();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Set up limit switches
  pinMode(PIN_LIMIT_NO, INPUT_PULLUP);
  pinMode(PIN_LIMIT_NC, INPUT_PULLUP);

  if (digitalRead(PIN_LIMIT_NC) && digitalRead(PIN_LIMIT_NO)) {
    errorAndStop("Limit switch not detected", LIMIT_SWITCH_INIT);
  }
  else if (!digitalRead(PIN_LIMIT_NC) && !digitalRead(PIN_LIMIT_NO)) {
    errorAndStop("Limit switch malfunction", LIMIT_SWITCH_INIT);
  }

  // Set up motor
  pinMode(PIN_MOTOR_PUMP, OUTPUT);
  pinMode(PIN_MOTOR_SUCK, OUTPUT);
  digitalWrite(PIN_MOTOR_PUMP, LOW);
  digitalWrite(PIN_MOTOR_SUCK, LOW);

  // Set up radio and packets
  clearPacketPayloads();
  packets[0][PKT_IDX_TEAM_NUM] = TEAM_NUM;
  packets[0][PKT_IDX_PROFILE_HALF] = 0;
  packets[1][PKT_IDX_TEAM_NUM] = TEAM_NUM;
  packets[1][PKT_IDX_PROFILE_HALF] = 1;
  initRadio();

  initPressureSensor();

  if (encoder.begin(PIN_ENCODER) != 0) {
    errorAndStop("Encoder init failed", ENCODER_INIT);
    pinMode(PIN_MOTOR_PUMP, OUTPUT);
    pinMode(PIN_MOTOR_SUCK, OUTPUT);
    digitalWrite(PIN_MOTOR_PUMP, LOW);
    digitalWrite(PIN_MOTOR_SUCK, LOW);
  }

  // Initialize tasks
  taskScheduler.init();
  taskScheduler.addTask(taskUpdateEncoder);
  taskScheduler.addTask(taskRecordPressure);
  taskScheduler.addTask(taskTransmitSurfacePressure);
  taskScheduler.addTask(taskTransmitPackets);

  // Enable encoder task (always running)
  taskUpdateEncoder.enable();

  startStageDeploySuck();
}

void loop() {
  // Run background tasks
  taskScheduler.execute();

  // Always check for radio commands
  bool shouldSubmerge = receiveCommand();

  // Handle overrides first
  if (overrideState != OverrideState::NoOverride) {
    handleOverride();
    return;
  }

  // Main state machine
  switch (stage) {
    case StageType::DeploySuck:
      // Check for stage completion
      if (millis() > stageTimeout || !digitalRead(PIN_LIMIT_NO)) {
        motorStop();
        encoder.resetPosition();  // Set zero position

        startStageDeployWait();
      }
      break;

    case StageType::DeployWait:
      // Check for stage completion
      if (millis() > stageTimeout) {
        startStageDeployPump();
      }
      break;

    case StageType::DeployPump:
      // Check for stage completion
      if (millis() > stageTimeout || isEmpty()) {
        startStageWaitDeploying();
      }
      break;

    case StageType::WaitDeploying:
      // Check for submerge command
      if (millis() > stageTimeout || shouldSubmerge) {
        taskTransmitSurfacePressure.disable();

        startStageDescending();
      }
      break;

    case StageType::Descending:
      hover();

      // Check for stage completion
      if (millis() > stageTimeout || countValidPackets() >= 10) {
        motorStop();

        startStagePump();
      }
      break;

    case StageType::Pump:
      // Check for stage completion
      if (millis() > stageTimeout || isEmpty()) {
        motorStop();

        startStageAscending();
      }
      break;

    case StageType::Ascending:
      // Check for stage completion
      if (millis() > stageTimeout || getDepth() < PRESSURE_SENSOR_VERTICAL_OFFSET + 0.01) {
        startStageWaitWaitTransmitting();
      }
      break;

    case StageType::WaitTransmitting:
      // Check for submerge command to start next profile
      if (millis() > stageTimeout || shouldSubmerge) {
        taskTransmitPackets.disable();

        startStageDescending();
      }
      break;

    default: Serial.println("Error: Undefined float stage"); break;
  }
}

// Stage transition functions
void startStageDeploySuck() {
  Serial.println("Stage: DeploySuck");
  stage = StageType::DeploySuck;
  stageTimeout = millis() + SUCK_MAX;

  setLedColor(COLOR_INITIALIZING);
  motorSuck();
}

void startStageDeployWait() {
  Serial.println("Stage: DeployWait");
  stage = StageType::DeployWait;
  stageTimeout = millis() + AFTER_HOME_WAIT;

  setLedColor(COLOR_INITIALIZING);
  motorStop();
}

void startStageDeployPump() {
  Serial.println("Stage: DeployPump");
  stage = StageType::DeployPump;
  stageTimeout = millis() + PUMP_MAX;

  setLedColor(COLOR_INITIALIZING);
  motorPump();
}

void startStageWaitDeploying() {
  Serial.println("Stage: WaitDeploying");
  stage = StageType::WaitDeploying;
  stageTimeout = millis() + RELEASE_MAX;

  setLedColor(COLOR_READY);
  motorStop();
  taskTransmitSurfacePressure.enable();
}

void startStageDescending() {
  Serial.println("Stage: Descending");
  stage = StageType::Descending;
  stageTimeout = millis() + DESCEND_TIME + HOLD_TIME;

  setLedColor(COLOR_DESCENDING);

  // Set up pressure logging
  clearPacketPayloads();
  packets[0][PKT_IDX_TEAM_NUM] = TEAM_NUM;
  packets[0][PKT_IDX_PROFILE_HALF] = 0;
  packets[1][PKT_IDX_TEAM_NUM] = TEAM_NUM;
  packets[1][PKT_IDX_PROFILE_HALF] = 1;
  taskRecordPressure.enable();

  // Init Kalman filter
  depthMean = 0;
  velMean = 0;
  depthVar = 1;
  velVar = 1;
  depthVelCovar = 0;
  lastKalmanUpdate = millis();

  // Init control system
  depthErrAcc = 0;
}

void startStagePump() {
  Serial.println("Stage: Pump");
  stage = StageType::Pump;
  stageTimeout = millis() + PUMP_MAX;

  setLedColor(COLOR_ASCENDING);
  motorPump();
}

void startStageAscending() {
  Serial.println("Stage: Ascending");
  stage = StageType::Ascending;
  stageTimeout = millis() + ASCEND_TIME;

  setLedColor(COLOR_ASCENDING);
  motorStop();
}

void startStageWaitWaitTransmitting() {
  Serial.println("Stage: WaitTransmitting");
  stage = StageType::WaitTransmitting;
  stageTimeout = millis() + TX_MAX_TIME;

  setLedColor(COLOR_READY);
  taskRecordPressure.disable();  // Stop recording pressure
  taskTransmitPackets.enable();
  motorStop();
}

// Task implementations
void updateEncoderCallback() { encoder.update(); }

float getPressure() {
  pressureSensor.read();
  float pressure = pressureSensor.pressure();

  while (pressure < 500 || pressure > 2000) {
    // Reject depth reading
    Serial.print("Rejected pressure reading ");
    Serial.println(pressure);
    pressureSensor.read();
    pressure = pressureSensor.pressure();
  }

  return pressure;
}

float getDepth() {
  pressureSensor.read();
  float pressure = getPressure();

  if (surfaceAverage == 0) {
    return pressureSensor.depth();
  }
  else {
    return (pressure - surfaceAverage) * MBAR_TO_METER_OF_HEAD + PRESSURE_SENSOR_VERTICAL_OFFSET;
  }
}

void transmitPacketsCallback() {
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

void recordPressureCallback() {
  uint32_t currentTime = millis();
  memcpy(packets[profileHalf] + packetIndex, &currentTime, sizeof(uint32_t));
  packetIndex += sizeof(uint32_t);

  float depth = getDepth();
  serialPrintf("Calculated depth: %f\n", depth);

  memcpy(packets[profileHalf] + packetIndex, &depth, sizeof(float));
  packetIndex += sizeof(float);

  if (profileHalf == 0 && packetIndex >= PKT_LEN) {
    profileHalf = 1;
    packetIndex = PKT_HEADER_LEN;
  }
}

void transmitSurfacePressureCallback() {
  pressureSensor.read();
  float pressure = getPressure();
  serialPrintf("Reading pressure at surface: %f\n", pressure);

  int intComponent = pressure;
  int fracComponent = trunc((pressure - intComponent) * 10000);
  char judgePacketBuffer[JUDGE_PKT_SIZE];
  snprintf(
    judgePacketBuffer, JUDGE_PKT_SIZE, "ROS:SINGLE:%d:%lu,%d.%04d\0", TEAM_NUM, millis(),
    intComponent, fracComponent);
  Serial.println(judgePacketBuffer);

  rf95.send((uint8_t*)judgePacketBuffer, strlen(judgePacketBuffer));
  rf95.waitPacketSent();

  // Update surface pressure average
  surfacePressures[surfacePressureIndex % AVERAGE_PRESSURE_LEN] = pressure;
  surfacePressureIndex++;

  surfaceAverage = 0;
  int totalReadings = min(AVERAGE_PRESSURE_LEN, surfacePressureIndex);
  for (int i = 0; i < totalReadings; i++) {
    surfaceAverage += surfacePressures[i];
  }
  surfaceAverage /= totalReadings;
}

// Motor control functions
void motorPump() {
  Serial.println("PUMPING");
  digitalWrite(PIN_MOTOR_PUMP, HIGH);
  digitalWrite(PIN_MOTOR_SUCK, LOW);
  motorState = MotorState::Pump;
}

void motorSuck() {
  Serial.println("SUCKING");
  digitalWrite(PIN_MOTOR_PUMP, LOW);
  digitalWrite(PIN_MOTOR_SUCK, HIGH);
  motorState = MotorState::Suck;
}

void motorStop() {
  Serial.println("STOPPING");
  digitalWrite(PIN_MOTOR_PUMP, HIGH);
  digitalWrite(PIN_MOTOR_SUCK, HIGH);
  motorState = MotorState::Stop;
}

void setMotorSpeed(float setpoint) {
  if (setpoint < -1) {
    setpoint = -1;
  }
  else if (setpoint > 1) {
    setpoint = 1;
  }

  if (setpoint > 0) {
    // Suck
    digitalWrite(PIN_MOTOR_PUMP, LOW);
    analogWrite(PIN_MOTOR_SUCK, setpoint * 255);
  }
  else if (setpoint < 0) {
    // Pump
    analogWrite(PIN_MOTOR_PUMP, -setpoint * 255);
    digitalWrite(PIN_MOTOR_SUCK, LOW);
  }
  else {
    motorStop();
  }
  motorState = MotorState::SpeedControl;
}

// Control loop
void updateKalmanFilter(float deltaTime, float measuredDepth, float measurementVar) {
  // Prediction step
  float newDepthMean = depthMean + velMean * deltaTime;
  float newDepthVar = depthVar + deltaTime * deltaTime * velVar + 2 * deltaTime * depthVelCovar;
  float newVelVar = velVar + VEL_PROCESS_NOISE;
  float newCov = depthVelCovar + deltaTime * velVar;

  depthMean = newDepthMean;
  depthVar = newDepthVar;
  velVar = newVelVar;
  depthVelCovar = newCov;

  // Measurement step
  float kalmanGainDepth = depthVar / (depthVar + measurementVar);
  float kalmanGainVel = depthVelCovar / (depthVar + measurementVar);

  newDepthMean = depthMean + kalmanGainDepth * (measuredDepth - depthMean);
  float newVelMean = velMean + kalmanGainVel * (measuredDepth - depthMean);

  newDepthVar = depthVar - kalmanGainDepth * depthVar;
  newVelVar = velVar - kalmanGainVel * depthVelCovar;
  newCov = depthVelCovar - kalmanGainVel * depthVar;

  depthMean = newDepthMean;
  velMean = newVelMean;
  depthVar = newDepthVar;
  velVar = newVelVar;
  depthVelCovar = newCov;
}

float deltaTime;
float depthErr;

void hover() {
  uint32_t updateTime = millis();
  deltaTime = (updateTime - lastKalmanUpdate) * 0.001;
  lastKalmanUpdate = updateTime;

  float raw_depth = getDepth();

  updateKalmanFilter(deltaTime, raw_depth, DEPTH_SENSOR_VAR);

  // Serial.print("Depth: ");
  // Serial.print(depthMean, 3);
  // Serial.print(" Vel: ");
  // Serial.println(velMean, 3);

  depthErr = TARGET_DEPTH - depthMean;

  // From -1 to 1, positive is suck (increase depth)
  float motorSetpoint = controlTest();

  if ((motorSetpoint > 0 && !digitalRead(PIN_LIMIT_NO)) || (motorSetpoint < 0 && isEmpty())) {
    motorStop();
  }
  else {
    setMotorSpeed(motorSetpoint);
  }
}

float controlTest() { return sin(millis() / 1000.0); }

float controlOriginal() {
  depthErrAcc += depthErr * deltaTime;
  if (depthErrAcc > DEPTH_ACC_MAX) {
    depthErr = DEPTH_ACC_MAX;
  }
  else if (depthErrAcc < -DEPTH_ACC_MAX) {
    depthErrAcc = -DEPTH_ACC_MAX;
  }

  float targetAngle =
    DEPTH_P * depthErr + DEPTH_I * depthErrAcc + DEPTH_D * -velMean + NEUTRAL_BOUYANCY_ANGLE;

  // targetAngle = NEUTRAL_BOUYANCY_ANGLE;

  return MOT_P * (targetAngle - getMotorAngle());
}

float controlVelocityBased() {
  float MAX_DEPTH_ERR = 1;

  float targetVelocity = constrain(depthErr, -MAX_DEPTH_ERR, MAX_DEPTH_ERR) * 0.1;

  return targetVelocity * 10;
}

float controlBangBang() {
  float tolerance = 0.1;

  if (depthMean < TARGET_DEPTH - tolerance) {
    return 1;
  }
  if (depthMean > TARGET_DEPTH + tolerance) {
    return -1;
  }
  return 0;
}

float controlPD() {
  float k_P = 0.5;
  float k_D = 20;

  float p_contrib = k_P * depthErr;
  p_contrib = constrain(p_contrib, -1, 1);
  return p_contrib - velMean * k_D;
}

// Command processing
bool receiveCommand() {
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

void handleOverride() {
  setLedColor(COLOR_ERROR);

  switch (overrideState) {
    case OverrideState::Suck:
      if (digitalRead(PIN_LIMIT_NO) == HIGH) {
        motorSuck();
      }
      else {
        motorStop();
        overrideState = OverrideState::Stop;
      }
      break;

    case OverrideState::Pump:
      if (!isEmpty()) {
        motorPump();
      }
      else {
        motorStop();
        overrideState = OverrideState::Stop;
      }
      break;

    case OverrideState::Stop: motorStop(); break;

    default: break;
  }

  // Check for commands to exit override
  receiveCommand();
}

// Utility functions
float getMotorAngle() { return -encoder.position * TWO_PI / COUNTS_PER_REV; }

bool isEmpty() { return getMotorAngle() > EMPTY_ANGLE; }

int countValidPackets() {
  float depths[2 * (int)((PKT_LEN - PKT_HEADER_LEN) / (sizeof(float) + sizeof(uint32_t)))];
  int totalPackets = 0;
  for (int half = 0; half < 2; half++) {
    for (int p = PKT_HEADER_LEN + sizeof(uint32_t); p < PKT_LEN;
         p += sizeof(float) + sizeof(uint32_t)) {
      memcpy(&depths[totalPackets], packets[half] + p, sizeof(float));
      totalPackets++;
    }
  }
  int validPackets = 0;
  for (int i = 0; i < totalPackets; i++) {
    if (TARGET_DEPTH - DEPTH_TOLERANCE < depths[i] && depths[i] < TARGET_DEPTH + DEPTH_TOLERANCE) {
      validPackets++;
    }
  }
  return validPackets;
}

void clearPacketPayloads() {
  for (int half = 0; half < 2; half++) {
    for (int i = PKT_HEADER_LEN; i < PKT_LEN; i++) {
      packets[half][i] = 0;
    }
  }
}

void setLedColor(COLOR color) {
  analogWrite(PIN_LED_RED, color.r);
  analogWrite(PIN_LED_GREEN, color.g);
  analogWrite(PIN_LED_BLUE, color.b);
}

void errorAndStop(String errorMsg, uint8_t error_code) {
  while (true) {
    Serial.println(errorMsg);

    for (uint8_t i = 0; i < error_code; i++) {
      setLedColor(COLOR_ERROR);
      delay(250);
      setLedColor(COLOR_OFF);
      delay(250);
    }

    delay(750);
  }
}

/******* Setup Methods *******/

void initRadio() {
  softwareSPI.setPins(8, 15, 14);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println("Feather RFM95 TX Test!");
  Serial.println();

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    errorAndStop("RFM95 radio init failed", RADIO_INIT);
  }
  Serial.println("RFM95 radio init OK!");

  if (!rf95.setFrequency(RADIO_FREQ)) {
    Serial.println("setFrequency failed");
  }

  rf95.setTxPower(23, false);

  Serial.print("RFM95 radio @ MHz: ");
  Serial.println(RADIO_FREQ);
}

void initPressureSensor() {
  setLedColor({0, 255, 255});
  bool init_success = false;

  Wire.begin();

  for (int i = 0; i < 5; i++) {
    if (pressureSensor.init()) {
      init_success = true;
      break;
    }
    delay(200);
  }

  if (!init_success) {
    errorAndStop("Pressure sensor init failed", PRESSURE_SENSOR_INIT);
  }

  pressureSensor.setModel(MS5837::MS5837_30BA);
  pressureSensor.setFluidDensity(997);
}