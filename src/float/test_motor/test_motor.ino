const uint8_t MOTOR_PWM_1 = 20;
const uint8_t MOTOR_PWM_2 = 21;

void setup() {
  Serial.begin(115200);
  // Wait until serial console is open; remove if not tethered to computer
  while (!Serial) {}

  Serial.println("Float Transceiver");
  Serial.println();

  pinMode(MOTOR_PWM_1, OUTPUT);
  pinMode(MOTOR_PWM_2, OUTPUT);

  digitalWrite(MOTOR_PWM_1, LOW);
  digitalWrite(MOTOR_PWM_2, LOW);
}

void loop() {
  Serial.println("Starting motor loop");

  // put your main code here, to run repeatedly:
  digitalWrite(MOTOR_PWM_2, LOW);

  for (int i = 60; i <= 255; i += 10) {
    Serial.print("Motoring at ");
    Serial.println(i);
    analogWrite(MOTOR_PWM_1, i);
    delay(500);
  }

  analogWrite(MOTOR_PWM_1, 0);

  Serial.println("Switch!");

  digitalWrite(MOTOR_PWM_1, LOW);

  for (int i = 0; i <= 255; i += 5) {
    Serial.print("Motoring at ");
    Serial.println(i);
    analogWrite(MOTOR_PWM_2, i);
    delay(500);
  }
}
