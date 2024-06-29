import rclpy
from rclpy.node import Node
from rov_msgs.msg import Flooding
import lgpio

# Pins used for GPIO
DETECT_PIN = 17


class FloodDetector(Node):

    def __init__(self) -> None:
        super().__init__('gpio_reader', parameter_overrides=[])
        self.publisher = self.create_publisher(Flooding, 'flooding', 10)
        self.gpio_chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_input(self.gpio_chip, DETECT_PIN)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self) -> None:
        # If any of the sensors detect water, send true to /tether/flooding
        flood_reading = lgpio.gpio_read(self.gpio_chip, DETECT_PIN)

        # If any of the sensors detect water, send true to /tether/flooding
        msg = Flooding()
        msg.flooding = (flood_reading == lgpio.HIGH)
        if msg.flooding:
            self.get_logger().error("The ebay is flooding")
        self.publisher.publish(msg)


def main(args: None = None) -> None:
    rclpy.init()
    flood_detector = FloodDetector()
    rclpy.spin(flood_detector)
    flood_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
