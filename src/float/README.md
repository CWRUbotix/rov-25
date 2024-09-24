# Setup

https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/setup

# Hardware

Adafruit Feather 32u4 RFM95 LoRa Radio- 868 or 915 MHz - RadioFruit

# Dependencies

RadioHead v1.122.1 by Mike McCauley

Blue Robotics MS5837 Library v1.1.1 by BlueRobotics

# Transceiver Arduino Sketches

These sketches must include files from `include/`, but that's impossible with the Arduino IDE. We hardlink the each file in `include/` manually.

If you add more files to `include/`, do something like this to hardlink them:

```bash
source remake_links.sh
```