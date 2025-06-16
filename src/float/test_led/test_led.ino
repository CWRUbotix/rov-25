#define LED_RED_PIN   9
#define LED_BLUE_PIN  6
#define LED_GREEN_PIN 12

struct COLOR {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

COLOR RED = {255, 0, 0};
COLOR GREEN = {0, 255, 0};
COLOR BLUE = {0, 0, 255};

void setup() {
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
}

void loop() {
  setLedColor(RED);
  delay(1000);
  setLedColor(BLUE);
  delay(1000);
  setLedColor(GREEN);
  delay(1000);
}

void setLedColor(COLOR color) {
  analogWrite(LED_RED_PIN, color.r);
  analogWrite(LED_GREEN_PIN, color.g);
  analogWrite(LED_BLUE_PIN, color.b);
}
