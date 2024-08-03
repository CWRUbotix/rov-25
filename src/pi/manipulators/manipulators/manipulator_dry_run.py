import time

from smbus2 import SMBus, i2c_msg

ADDRESS = 0x20


def main() -> None:
    with SMBus(1) as bus:
        while True:
            print('on')
            write_all = i2c_msg.write(ADDRESS, [0x06, 0b00000000])

            bus.i2c_rdwr(write_all)

            time.sleep(2)
            print('off')

            a = i2c_msg.write(ADDRESS, [0x06, 0b11111111])

            bus.i2c_rdwr(a)
            time.sleep(2)


if __name__ == '__main__':
    main()
