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
#include <Wire.h>
#include <TaskScheduler.h>

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
#define PIN_LED_RED 9
#define PIN_LED_BLUE 6
#define PIN_LED_GREEN 12

// The encoder requires two adjacent pins on the PIO
#define PIN_ENCODER 4

const uint8_t TEAM_NUM = 25;
const uint32_t PACKET_SEND_INTERVAL = 1000;
const uint32_t FLOAT_PKT_RX_TIMEOUT = 10;
const uint8_t JUDGE_PKT_SIZE = 30;

#ifdef DO_DEBUGGING
const uint32_t PRESSURE_READ_INTERVAL = 200;
const uint32_t PROFILE_SEGMENT = 10000;
#else
const uint32_t PRESSURE_READ_INTERVAL = 5000;
const uint32_t PROFILE_SEGMENT = 60000;
#endif

const uint32_t ONE_MINUTE = 60000;

// Schedule timings (all delays in ms)
const uint32_t AFTER_HOME_WAIT = 10'000;
const uint32_t RELEASE_MAX = 8 * ONE_MINUTE;
const uint32_t SUCK_MAX = PROFILE_SEGMENT;
const uint32_t DESCEND_TIME = PROFILE_SEGMENT;
const uint32_t PUMP_MAX = PROFILE_SEGMENT;
const uint32_t ASCEND_TIME = 0;
const uint32_t TX_MAX_TIME = 2 * ONE_MINUTE;
const uint32_t HOLD_TIME = 0;

// Distance from the pressure sensor to the bottom of the float (m)
const float MBAR_TO_METER_OF_HEAD = 0.010199773339984;
const float PRESSURE_SENSOR_VERTICAL_OFFSET = 0.635;
const int AVERAGE_PRESSURE_LEN = 5;

float surfacePressures[AVERAGE_PRESSURE_LEN];
float surfaceAverage = 0;
int surfacePressureIndex = 0;

RHSoftwareSPI softwareSPI;
PicoEncoder encoder;
long EMPTY_POS = -4'500'000;

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
enum class MotorState { Stop, Suck, Pump, Error };

StageType stage = StageType::DeploySuck;
OverrideState overrideState = OverrideState::NoOverride;
MotorState motorState = MotorState::Error;

uint8_t profileNum = 0;
uint8_t profileHalf = 0;

// Timing variables
uint32_t stageStartTime = 0;
uint32_t stageTimeout = 0;
uint32_t nextPacketTransmitTime = 0;
uint32_t nextSurfacePressureTime = 0;

// Colors for status LED
struct COLOR {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

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

// Task scheduler and task declarations
Scheduler taskScheduler;

// Task callback function declarations
void updateEncoderCallback();
void recordPressureCallback();

// Task objects
Task taskUpdateEncoder(10, TASK_FOREVER, &updateEncoderCallback);
Task taskRecordPressure(PRESSURE_READ_INTERVAL, TASK_FOREVER, &recordPressureCallback);

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
  } else if (!digitalRead(PIN_LIMIT_NC) && !digitalRead(PIN_LIMIT_NO)) {
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
  }

  // Initialize tasks
  taskScheduler.init();
  taskScheduler.addTask(taskUpdateEncoder);
  taskScheduler.addTask(taskRecordPressure);

  // Enable encoder task (always running)
  taskUpdateEncoder.enable();

  // Initialize stage timing
  stageStartTime = millis();
  stageTimeout = stageStartTime + SUCK_MAX;
  nextSurfacePressureTime = millis() + PRESSURE_READ_INTERVAL;

  // Start first stage
  setLedColor(COLOR_INITIALIZING);
  motor_suck();
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
        motor_stop();
        encoder.resetPosition();  // Set zero position
        
        // Transition to DeployWait
        stage = StageType::DeployWait;
        stageStartTime = millis();
        stageTimeout = stageStartTime + AFTER_HOME_WAIT;
        setLedColor(COLOR_INITIALIZING);
        Serial.println("Stage: DeployWait");
      }
      break;

    case StageType::DeployWait:
      // Check for stage completion
      if (millis() > stageTimeout) {
        
        // Transition to DeployPump
        stage = StageType::DeployPump;
        stageStartTime = millis();
        stageTimeout = stageStartTime + PUMP_MAX;
        setLedColor(COLOR_INITIALIZING);
        motor_pump();
        Serial.println("Stage: DeployPump");
      }
      break;

    case StageType::DeployPump:
      // Check for stage completion
      if (millis() > stageTimeout || isEmpty()) {
        // Transition to WaitDeploying
        stage = StageType::WaitDeploying;
        stageStartTime = millis();
        stageTimeout = stageStartTime + RELEASE_MAX;
        setLedColor(COLOR_READY);
        motor_stop();
        Serial.println("Stage: WaitDeploying");
      }
      break;

    case StageType::WaitDeploying:
      // Handle surface pressure transmission
      if (millis() >= nextSurfacePressureTime) {
        transmitSurfacePressure();
        nextSurfacePressureTime = millis() + PRESSURE_READ_INTERVAL;
      }

      // Check for submerge command
      if (millis() > stageTimeout || shouldSubmerge) {
        // Transition to Descending
        stage = StageType::Descending;
        stageStartTime = millis();
        stageTimeout = stageStartTime + DESCEND_TIME + HOLD_TIME;
        setLedColor(COLOR_DESCENDING);
        motor_suck();
        taskRecordPressure.enable();  // Start recording pressure
        Serial.println("Stage: Descending");
      }
      break;

    case StageType::Descending:
      // TODO: Fancy PID stuff
      if (!digitalRead(PIN_LIMIT_NO)) {
        motor_stop();
      }

      // Check for stage completion
      if (millis() > stageTimeout) {
        motor_stop();
        
        // Transition to Pump
        stage = StageType::Pump;
        stageStartTime = millis();
        stageTimeout = stageStartTime + PUMP_MAX;
        setLedColor(COLOR_ASCENDING);
        motor_pump();
        Serial.println("Stage: Pump");
      }
      break;

    case StageType::Pump:
      // Check for stage completion
      if (millis() > stageTimeout || isEmpty()) {
        // Transition to Ascending
        stage = StageType::Ascending;
        stageStartTime = millis();
        stageTimeout = stageStartTime + ASCEND_TIME;
        setLedColor(COLOR_ASCENDING);
        motor_stop();
        Serial.println("Stage: Ascending");
      }
      break;

    case StageType::Ascending:
      // Check for stage completion (immediate since ASCEND_TIME = 0)
      if (millis() > stageTimeout) {
        // Transition to WaitTransmitting
        stage = StageType::WaitTransmitting;
        stageStartTime = millis();
        stageTimeout = stageStartTime + TX_MAX_TIME;
        setLedColor(COLOR_READY);
        taskRecordPressure.disable();  // Stop recording pressure
        nextPacketTransmitTime = millis();  // Transmit immediately
        Serial.println("Stage: WaitTransmitting");
      }
      break;

    case StageType::WaitTransmitting:
      // Handle packet transmission
      if (millis() >= nextPacketTransmitTime) {
        transmitPressurePacket();
        nextPacketTransmitTime = millis() + PACKET_SEND_INTERVAL;
      }

      // Check for submerge command to start next profile
      if (millis() > stageTimeout || shouldSubmerge) {
        // Transition back to Descending
        stage = StageType::Descending;
        stageStartTime = millis();
        stageTimeout = stageStartTime + DESCEND_TIME + HOLD_TIME;
        setLedColor(COLOR_DESCENDING);
        motor_suck();
        taskRecordPressure.enable();  // Start recording pressure again
        Serial.println("Stage: Descending (next profile)");
      }
      break;

    default:
      Serial.println("Error: Undefined float stage");
      break;
  }
}

// Task implementations
void updateEncoderCallback() {
  encoder.update();
}

void recordPressureCallback() {
  uint32_t currentTime = millis();
  memcpy(packets[profileHalf] + packetIndex, &currentTime, sizeof(uint32_t));
  packetIndex += sizeof(uint32_t);

  pressureSensor.read();
  float pressure = pressureSensor.pressure();
  serialPrintf("Reading pressure: %f\n", pressure);

  float depth;
  if (surfaceAverage == 0) {
    depth = pressureSensor.depth();
  } else {
    depth = (pressure - surfaceAverage) * MBAR_TO_METER_OF_HEAD + PRESSURE_SENSOR_VERTICAL_OFFSET;
  }
  serialPrintf("Calculated depth: %f\n", depth);

  memcpy(packets[profileHalf] + packetIndex, &depth, sizeof(float));
  packetIndex += sizeof(float);

  if (profileHalf == 0 && packetIndex >= PKT_LEN) {
    profileHalf = 1;
    packetIndex = PKT_HEADER_LEN;
  }
}

// Motor control functions
void motor_pump() {
  Serial.println("PUMPING");
  digitalWrite(PIN_MOTOR_PUMP, HIGH);
  digitalWrite(PIN_MOTOR_SUCK, LOW);
  motorState = MotorState::Pump;
}

void motor_suck() {
  Serial.println("SUCKING");
  digitalWrite(PIN_MOTOR_PUMP, LOW);
  digitalWrite(PIN_MOTOR_SUCK, HIGH);
  motorState = MotorState::Suck;
}

void motor_stop() {
  Serial.println("STOPPING");
  digitalWrite(PIN_MOTOR_PUMP, HIGH);
  digitalWrite(PIN_MOTOR_SUCK, HIGH);
  motorState = MotorState::Stop;
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
        motor_suck();
      } else {
        motor_stop();
        overrideState = OverrideState::Stop;
      }
      break;
      
    case OverrideState::Pump:
      if (!isEmpty()) {
        motor_pump();
      } else {
        motor_stop();
        overrideState = OverrideState::Stop;
      }
      break;
      
    case OverrideState::Stop:
      motor_stop();
      break;
      
    default:
      break;
  }
  
  // Check for commands to exit override
  receiveCommand();
}

void transmitSurfacePressure() {
  pressureSensor.read();
  float pressure = pressureSensor.pressure();
  serialPrintf("Reading pressure at surface: %f\n", pressure);

  int intComponent = pressure;
  int fracComponent = trunc((pressure - intComponent) * 10000);
  char judgePacketBuffer[JUDGE_PKT_SIZE];
  snprintf(judgePacketBuffer, JUDGE_PKT_SIZE, "ROS:SINGLE:%d:%lu,%d.%04d\0", 
           TEAM_NUM, millis(), intComponent, fracComponent);
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

// Utility functions
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
    if (2 < depths[i] && depths[i] < 3) {
      validPackets++;
    }
  }
  return validPackets;
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

bool isEmpty() {
  return encoder.position < EMPTY_POS;
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

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
  }

  rf95.setTxPower(23, false);

  Serial.print("RFM95 radio @ MHz: ");
  Serial.println(RF95_FREQ);
}

void initPressureSensor() {
  Wire.begin();
  
  setLedColor({0, 255, 255});
  bool init_success = false;
  for(int i = 0; i < 5; i++) {
    if (pressureSensor.init()) {
      init_success = true;
      break;
    }
    delay(200);
  }

  if(!init_success) {
    errorAndStop("Pressure sensor init failed", PRESSURE_SENSOR_INIT);
  }

  pressureSensor.setModel(MS5837::MS5837_02BA);
  pressureSensor.setFluidDensity(997);

  serialPrintf("PRESSURE: %f\n", pressureSensor.pressure());
  pressureSensor.depth();
}