from time import sleep

import tsys01  # https://github.com/bluerobotics/tsys01-python


def main() -> None:
    sensor = tsys01.TSYS01()  # Use default I2C bus 1

    sensor.init()

    while True:
        try:
            sensor.read()  # Sometimes throws OSError: [Errno 121] Remote I/O error
            print(
                sensor.temperature(),  # Get temperature in default units (Centigrade)
                '\t',
                sensor.temperature(tsys01.UNITS_Farenheit)
            )
        except OSError:
            print('Failed to read temperature, trying again')

        sleep(1)


if __name__ == "__main__":
    main()
