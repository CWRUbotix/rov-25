import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
import tsys01

from rov_msgs.msg import Temperature

READING_TIMER_PERIOD = 0.5  # Seconds


class TempSensor(Node):

    def __init__(self) -> None:
        super().__init__('temp_sensor', parameter_overrides=[])
        self.publisher = self.create_publisher(Temperature, 'temperature',
                                               QoSPresetProfiles.DEFAULT.value)
        self.sensor = tsys01.TSYS01()
        self.sensor.init()

        self.timer = self.create_timer(READING_TIMER_PERIOD, self.timer_callback)

    def timer_callback(self) -> None:
        try:
            self.sensor.read()
            temp_reading = self.sensor.temperature()

            # If any of the sensors detect water, send true to /tether/flooding
            self.publisher.publish(Temperature(reading=temp_reading))
        except OSError:
            print('Failed to read temperature, skipping this read')


def main(args: None = None) -> None:
    rclpy.init()
    temp_sensor = TempSensor()
    rclpy.spin(temp_sensor)
    temp_sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
