import time
from queue import Queue
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from serial import Serial
from serial.serialutil import SerialException

from rov_msgs.msg import FloatCommand, FloatData, FloatSerial, FloatSingle

MILLISECONDS_TO_SECONDS = 1/1000
SECONDS_TO_MINUTES = 1/60
MBAR_TO_METER_OF_HEAD = 0.010199773339984


AMBIENT_PRESSURE_DEFAULT = 1013.25  # in (mbar)

# Distance from the pressure sensor to the bottom of the float (m)
PRESSURE_SENSOR_VERTICAL_OFFSET = 0.635

AVERAGE_QUEUE_LEN = 5

ROS_PACKET = "ROS:"
ROS_SINGLE = ROS_PACKET + "SINGLE"
SECTION_SEPARATOR = ":"
DATA_SEPARATOR = ";"
COMMA_SEPARATOR = ","
HEADER_LENGTH = 3
PACKET_SECTIONS = 3


class SerialReader(Node):

    def __init__(self) -> None:
        super().__init__('serial_reader')
        self.data_publisher = self.create_publisher(FloatData, 'transceiver_data',
                                                    QoSPresetProfiles.SENSOR_DATA.value)

        self.ros_single_publisher = self.create_publisher(FloatSingle, 'transceiver_single',
                                                          QoSPresetProfiles.SENSOR_DATA.value)

        self.create_subscription(FloatCommand, 'float_command', self.send_command,
                                 QoSPresetProfiles.DEFAULT.value)

        self.serial_publisher = self.create_publisher(FloatSerial, 'float_serial',
                                                      QoSPresetProfiles.SENSOR_DATA.value)

        self.serial_packet_handler = SerialReaderPacketHandler()

        try:
            self.serial = Serial("/dev/serial/by-id/usb-Adafruit_Feather_32u4-if00", 115200)
        except SerialException:
            self.get_logger().error("Could not get serial device")
            exit(1)

        Thread(target=self.read_serial, daemon=True,
               name="Serial Reader").start()

    def send_command(self, msg: FloatCommand) -> None:
        self.serial.write(msg.command.encode())

    def read_serial(self) -> None:
        buffer = b''
        while True:
            buffer += self.serial.read(self.serial.in_waiting)

            while b'\n' in buffer:
                packet, buffer = buffer.split(b'\n', 1)
                self.ros_publish(packet.decode())
            time.sleep(0.05)

    def ros_publish(self, packet: str) -> None:
        """Publish a message from the transceiver."""
        self.serial_publisher.publish(FloatSerial(serial=packet))

        if packet[:len(ROS_PACKET)] != ROS_PACKET:
            return

        try:
            if SerialReaderPacketHandler.is_ros_single_message(packet):
                single_msg = self.serial_packet_handler.handle_ros_single(packet)
                if single_msg:
                    self.ros_single_publisher.publish(single_msg)
            else:
                msg = self.serial_packet_handler.message_parser(packet)
                self.data_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error {e} caught dropping packet")


class SerialReaderPacketHandler:

    def __init__(self, queue_size: int = AVERAGE_QUEUE_LEN) -> None:
        self.surface_pressure = AMBIENT_PRESSURE_DEFAULT
        self.surface_pressures: Queue[float] = Queue(queue_size)

    @staticmethod
    def is_ros_single_message(packet: str) -> bool:
        return packet[:len(ROS_SINGLE)] == ROS_SINGLE

    def handle_ros_single(self, packet: str) -> FloatSingle:
        packet_sections = packet.split(SECTION_SEPARATOR)
        team_number = int(packet_sections[2])
        data = packet_sections[3]
        time_ms = int(data.split(COMMA_SEPARATOR)[0])
        pressure = float(data.split(COMMA_SEPARATOR)[1])

        float_msg = FloatSingle(team_number=team_number,
                                time_ms=time_ms,
                                pressure=pressure)

        if not self.surface_pressures.full():
            self.surface_pressures.put(pressure)

            iterable_queue = self.surface_pressures.queue
            avg_pressure = sum(iterable_queue) / len(iterable_queue)
            self.surface_pressure = avg_pressure

        if self.surface_pressures.full():
            float_msg.average_pressure = self.surface_pressure

        return float_msg

    def message_parser(self, packet: str) -> FloatData:
        msg = FloatData()

        packet_sections = packet.split(SECTION_SEPARATOR)

        if len(packet_sections) != PACKET_SECTIONS:
            raise ValueError(f"Packet expected {PACKET_SECTIONS} sections, "
                             f"found {len(packet_sections)} sections")

        header = packet_sections[1].split(COMMA_SEPARATOR)
        data = packet_sections[2]

        if len(header) != HEADER_LENGTH:
            raise ValueError(f"Packet header length of {HEADER_LENGTH} expected "
                             f"found {len(header)} instead")

        msg.team_number = int(header[0])
        msg.profile_number = int(header[1])
        msg.profile_half = int(header[2])

        time_data_list: list[float] = []
        depth_data_list: list[float] = []

        for time_reading, pressure_reading in [data.split(COMMA_SEPARATOR) for data in
                                               data.split(DATA_SEPARATOR)]:

            if int(time_reading) == 0:
                continue
            # Starts out as uint32
            time_data_list.append(int(time_reading) * MILLISECONDS_TO_SECONDS * SECONDS_TO_MINUTES)

            # Starts out as float
            depth_data_list.append(
                (float(pressure_reading) - self.surface_pressure) * MBAR_TO_METER_OF_HEAD
                + PRESSURE_SENSOR_VERTICAL_OFFSET
            )

        msg.time_data = time_data_list
        msg.depth_data = depth_data_list

        return msg


def main() -> None:
    """Run the serial reader node."""
    rclpy.init()

    serial_reader = SerialReader()
    rclpy.spin(serial_reader)


if __name__ == '__main__':
    main()
