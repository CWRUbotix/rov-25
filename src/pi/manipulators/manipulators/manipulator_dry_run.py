
from tca9555 import TCA9555
import time

ALL_BITS = (0, 1, 2, 3, 4, 5)


def main() -> None:
    # Initialize with standard I2C-bus address of TCA9555 a.k.a 0x20
    gpio = TCA9555()  # can put in the address as a param in hexadecimal

    # # Print :startup-config as human-readable
    print(gpio.format_config())

    # # Set pins 0 through 5 as output
    gpio.set_direction(0, bits=ALL_BITS)
    print(gpio.format_config())

    try:
        while True:
            # Turn on the LEDs
            gpio.set_bits(bits=ALL_BITS)
            print(gpio.format_config())
            time.sleep(2)

            # # Turn off the LEDs
            gpio.unset_bits(bits=ALL_BITS)
            print(gpio.format_config())
            time.sleep(2)
    except KeyboardInterrupt:
        gpio.unset_bits(bits=ALL_BITS)


if __name__ == "__main__":
    main()
