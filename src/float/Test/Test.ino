#include <RHSoftwareSPI.h>
#include <RH_RF95.h>
#include <SPI.h>

#include "MS5837.h"
#include "PicoEncoder.h"
#include "Wire.h"

// #if defined(__AVR_ATmega32U4__)  // Feather 32u4 w/Radio
// const uint8_t RFM95_CS = 8;
// const uint8_t RFM95_INT = 7;
// const uint8_t RFM95_RST = 4;

// #elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || \
//   defined(ARDUINO_SAMD_FEATHER_M0)  // Feather M0 w/Radio
// const uint8_t RFM95_CS = 8;
// const uint8_t RFM95_INT = 3;
// const uint8_t RFM95_RST = 4;

// #elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_RFM)  // Feather RP2040 w/Radio
// const uint8_t RFM95_CS = 16;
// const uint8_t RFM95_INT = 21;
// const uint8_t RFM95_RST = 17;

// #elif defined(__AVR_ATmega328P__)  // Feather 328P w/wing
// const uint8_t RFM95_CS = 4;
// const uint8_t RFM95_INT = 3;
// const uint8_t RFM95_RST = 2;  // "A"

// #elif defined(ESP8266)  // ESP8266 feather w/wing
// const uint8_t RFM95_CS = 2;    // "E"
// const uint8_t RFM95_INT = 15;  // "B"
// const uint8_t RFM95_RST = 16;  // "D"

// #elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_NRF52840_FEATHER) || \
//   defined(ARDUINO_NRF52840_FEATHER_SENSE)
// const uint8_t RFM95_CS = 10;   // "B"
// const uint8_t RFM95_INT = 9;   // "A"
// const uint8_t RFM95_RST = 11;  // "C"

// #elif defined(ESP32)  // ESP32 feather w/wing
// const uint8_t RFM95_CS = 33;   // "B"
// const uint8_t RFM95_INT = 27;  // "A"
// const uint8_t RFM95_RST = 13;

// #elif defined(ARDUINO_NRF52832_FEATHER)  // nRF52832 feather w/wing
// const uint8_t RFM95_CS = 11;   // "B"
// const uint8_t RFM95_INT = 31;  // "C"
// const uint8_t RFM95_RST = 7;   // "A"

// #else  // Custom RP2040 setup
const uint8_t RFM95_CS = 16;
const uint8_t RFM95_INT = 22;
const uint8_t RFM95_RST = 17;

// #endif

// const uint8_t MOTOR_PWM = 6;  // Leave 100% cycle for top speed
// const uint8_t PUMP_PIN = 9;   // Set high for pump (CW when facing down)
// const uint8_t SUCK_PIN = 10;  // Set high for suck (CCW when facing down)

const uint8_t MOTOR_PWM_1 = 20;
const uint8_t MOTOR_PWM_2 = 21;

const uint8_t LIMIT_FULL = 12;   // Low when syringe is full
const uint8_t LIMIT_EMPTY = 11;  // Low when syringe is empty

const float RF95_FREQ = 877.0;

// Encoder initialization
PicoEncoder encoder;
const uint8_t ENCODER_PIN = 4;

const int TOLERANCE = 0;

RHSoftwareSPI softwareSPI;

// Singleton instance of  the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT, softwareSPI);

// Singleton instance of the pressure sensor driver
MS5837 pressureSensor;

void setup() {
  delay(2000);
  Serial.begin(115200);
  Wire.begin();  // Initialize I2C communication
  // Wait until serial console is open; remove if not tethered to computer
  while (!Serial) {}

  Serial.println("Float Transceiver");
  Serial.println();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(LIMIT_EMPTY, INPUT_PULLUP);
  pinMode(LIMIT_FULL, INPUT_PULLUP);

  // pinMode(MOTOR_PWM, OUTPUT);
  // pinMode(SUCK_PIN, OUTPUT);
  // pinMode(PUMP_PIN, OUTPUT);

  pinMode(MOTOR_PWM_1, OUTPUT);
  pinMode(MOTOR_PWM_2, OUTPUT);

  digitalWrite(MOTOR_PWM_1, LOW);
  digitalWrite(MOTOR_PWM_2, LOW);

  // digitalWrite(MOTOR_PWM, LOW);
  // digitalWrite(SUCK_PIN, LOW);
  // digitalWrite(PUMP_PIN, LOW);

  initRadio();
  initPressureSensor();
  encoder.begin(ENCODER_PIN);
}

void loop() {
  // Serial.println("Starting motor loop");

  // put your main code here, to run repeatedly:
  String data = "Hello";
  byte dataBytes[data.length() + 1];
  data.getBytes(dataBytes, data.length() + 1);
  rf95.send(dataBytes, data.length() + 1);

  delay(1000);
  pressureSensor.read();
  Serial.println(pressureSensor.pressure());
  Serial.println(pressureSensor.temperature());
  // while (!travelTo(1920));
  // delay(2000);
  // encoder.update();
  // Serial.println((int)encoder.step);
  // digitalWrite(MOTOR_PWM_2, LOW);

  // for (int i = 60; i <= 255; i += 10) {
  //   Serial.print("Motoring at ");
  //   Serial.println(i);
  //   analogWrite(MOTOR_PWM_1, i);
  //   delay(500);
  // }

  // analogWrite(MOTOR_PWM_1, 0);

  // Serial.println("Switch!");

  // digitalWrite(MOTOR_PWM_1, LOW);

  // for (int i = 0; i <= 255; i += 5) {
  //   Serial.print("Motoring at ");
  //   Serial.println(i);
  //   analogWrite(MOTOR_PWM_2, i);
  //   delay(500);
  // }
}

// Instruct the motor to travel to a position
bool travelTo(int pos) {
  pos = pos * 1920 / 360;
  encoder.update();
  // Serial.println((int)encoder.step);
  if (pos < (int)encoder.step - TOLERANCE) {
    analogWrite(MOTOR_PWM_1, 60);
    digitalWrite(MOTOR_PWM_2, LOW);
    return false;
  }
  else if (pos > (int)encoder.step + TOLERANCE) {
    digitalWrite(MOTOR_PWM_1, LOW);
    analogWrite(MOTOR_PWM_2, 60);
    return false;
  }
  else {
    digitalWrite(MOTOR_PWM_1, HIGH);
    digitalWrite(MOTOR_PWM_2, HIGH);
    return true;
  }
}

void initRadio() {
  softwareSPI.setPins(8, 15, 14);

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

  Serial.print("RFM95 radio @ MHz: ");
  Serial.println(RF95_FREQ);
}

void initPressureSensor() {
  while (!pressureSensor.init()) {
    Serial.println("Waiting for Pressure Sensor init...");
    delay(5000);
  }
  pressureSensor.setModel(MS5837::MS5837_02BA);
  pressureSensor.setFluidDensity(997);  // kg/m^3

  Serial.print("PRESSURE: ");
  Serial.println(pressureSensor.pressure());
}
