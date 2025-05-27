#include "PicoEncoder.h"

// The encoder requires two adjecent pins on the PIO, with the given pin being the lower-numbered
#define PIN_ENCODER 4

#define PIN_MOTOR_PUMP 20
#define PIN_MOTOR_SUCK 21

PicoEncoder encoder;
const int STEP_TOL = 10000;

void setup() {
  pinMode(PIN_MOTOR_PUMP, OUTPUT);
  pinMode(PIN_MOTOR_SUCK, OUTPUT);

  delay(2000);

  Serial.begin(115200);

  if (encoder.begin(PIN_ENCODER) != 0) {
    Serial.print("Encoder init failed!");
    while (true);
  }
}

void loop() {
  encoder.update();
  Serial.println(encoder.position);

  travelTo(-500000);
}

// Instruct the motor to travel to a position
bool travelTo(int pos) {
  encoder.update();
  if (pos < (int)encoder.position - STEP_TOL) {
    // Pump
    digitalWrite(PIN_MOTOR_PUMP, HIGH);
    digitalWrite(PIN_MOTOR_SUCK, LOW);
    return false;
  }
  else if (pos > (int)encoder.position + STEP_TOL) {
    // Suck
    digitalWrite(PIN_MOTOR_PUMP, LOW);
    digitalWrite(PIN_MOTOR_SUCK, HIGH);
    return false;
  }
  else {
    // Off
    digitalWrite(PIN_MOTOR_PUMP, LOW);
    digitalWrite(PIN_MOTOR_SUCK, LOW);
    return true;
  }
}
