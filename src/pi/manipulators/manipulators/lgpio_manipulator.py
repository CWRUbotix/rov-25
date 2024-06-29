import lgpio
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rov_msgs.msg import Manip

# Configuration
MANIP_PIN_ONE = 23
MANIP_PIN_TWO = 24


class Manipulator(Node):

    def __init__(self) -> None:
        super().__init__('lgpio_manipulator')

        self.create_subscription(
            Manip,
            'manipulator_control',
            self.manip_callback,
            qos_profile_system_default
        )

        self.gpio_handle = lgpio.gpiochip_open(0)

        lgpio.gpio_claim_output(self.gpio_handle, MANIP_PIN_ONE)
        lgpio.gpio_claim_output(self.gpio_handle, MANIP_PIN_TWO)

    def manip_callback(self, message: Manip) -> None:
        manip_id = message.manip_id
        activated = message.activated

        if manip_id == "left":
            if activated:
                lgpio.gpio_write(self.gpio_handle, MANIP_PIN_ONE, lgpio.HIGH)
            else:
                lgpio.gpio_write(self.gpio_handle, MANIP_PIN_ONE, lgpio.LOW)
        elif manip_id == "right":
            if activated:
                lgpio.gpio_write(self.gpio_handle, MANIP_PIN_TWO, lgpio.HIGH)
            else:
                lgpio.gpio_write(self.gpio_handle, MANIP_PIN_TWO, lgpio.LOW)


def main() -> None:
    rclpy.init()

    subscriber = Manipulator()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
