# Transceiver Arduino Sketches

These sketches must include files from `include/`, but that's impossible with the Arduino IDE. We hardlink the each file in `include/` manually.

If you add more files to `include/`, do something like this to hardlink them:

```bash
source remake_links.sh
```