#include <Arduino.h>
#include <stdarg.h>

void serialPrintf(const char* input...) {
  va_list args;
  va_start(args, input);
  for (const char* c = input; *c != 0; c++) {
    if (*c != '%') {
      Serial.print(*c);
      continue;
    }
    c++;
    switch (*c) {
      case '%': Serial.print('%'); break;
      case 's': Serial.print(va_arg(args, char*)); break;
      case 'l': Serial.print(va_arg(args, uint32_t)); break;
      case 'd': Serial.print(va_arg(args, int), DEC); break;
      case 'b': Serial.print(va_arg(args, int), BIN); break;
      case 'o': Serial.print(va_arg(args, int), OCT); break;
      case 'x': Serial.print(va_arg(args, int), HEX); break;
      case 'f': Serial.print(va_arg(args, double), 2); break;
    }
  }
  // Serial.println();
  va_end(args);
}
