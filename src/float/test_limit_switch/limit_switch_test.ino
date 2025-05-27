#define PIN_LIMIT_NO 11
#define PIN_LIMIT_NC 10

#define PIN_LED_RED 9
#define PIN_LED_BLUE 6
#define PIN_LED_GREEN 12

void setup() {
  pinMode(PIN_LIMIT_NC, INPUT_PULLUP);
  pinMode(PIN_LIMIT_NO, INPUT_PULLUP);

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
}

void loop() {
  digitalWrite(PIN_LED_GREEN, !digitalRead(PIN_LIMIT_NC));
  digitalWrite(PIN_LED_RED, !digitalRead(PIN_LIMIT_NO));
}
