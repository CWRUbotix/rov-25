import lgpio
import time

# Configuration
SERVO_PIN = 12  # pin used to drive PWM fan
FREQ = 50

gpio_handle = lgpio.gpiochip_open(0)


def test_gpio(width: int, freq: int = 50) -> None:
    lgpio.tx_servo(gpio_handle, SERVO_PIN, width, freq)


def main() -> None:
    try:
        print('Starting loop')
        while True:
            print('one way')
            test_gpio(1900, 50)
            time.sleep(2)

            print('off')
            test_gpio(1500, 200)
            time.sleep(2)

            print('other way')
            test_gpio(1100, 400)
            time.sleep(2)

    except KeyboardInterrupt:
        # Turn the fan to medium speed
        lgpio.tx_pwm(gpio_handle, SERVO_PIN, FREQ, 50)
        lgpio.gpiochip_close(gpio_handle)


if __name__ == "__main__":
    main()
