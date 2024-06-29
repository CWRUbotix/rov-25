import time

import lgpio

# Configuration
MANIP_PIN_ONE = 23
MANIP_PIN_TWO = 24

gpio_handle = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(gpio_handle, MANIP_PIN_ONE)
lgpio.gpio_claim_output(gpio_handle, MANIP_PIN_TWO)


def main() -> None:
    try:
        print('Starting loop')
        while True:
            print("On")
            lgpio.gpio_write(gpio_handle, MANIP_PIN_ONE, lgpio.HIGH)
            lgpio.gpio_write(gpio_handle, MANIP_PIN_TWO, lgpio.HIGH)

            time.sleep(2)
            print('off')
            lgpio.gpio_write(gpio_handle, MANIP_PIN_ONE, lgpio.LOW)
            lgpio.gpio_write(gpio_handle, MANIP_PIN_TWO, lgpio.LOW)
            time.sleep(2)

    except KeyboardInterrupt:
        lgpio.gpio_write(gpio_handle, MANIP_PIN_ONE, lgpio.LOW)
        lgpio.gpio_write(gpio_handle, MANIP_PIN_TWO, lgpio.LOW)


if __name__ == "__main__":
    main()
