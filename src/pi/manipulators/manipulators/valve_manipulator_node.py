from typing import Final

import lgpio
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSPresetProfiles
from mavros_msgs.msg import ManualControl

from rov_msgs.msg import ValveManip

# Configuration
SERVO_PIN: Final = 12  # pin used to drive Valve Manip

EXTENSIONS_CODE: Final = 0b00000100

class ValveManipulator(Node):
    def __init__(self) -> None:
        super().__init__('valve_manipulator')
        self.create_subscription(
            ValveManip, 'valve_manipulator', self.manip_callback, qos_profile_system_default
        )

        self.gpio_handle = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.gpio_handle, SERVO_PIN)
        self.curr_active = False

    def servo(self, width: int, freq: int = 50) -> None:
        # mc_msg = ManualControl()
        # mc_msg.enabled_extensions = EXTENSIONS_CODE
        # mc_msg.aux1 = 1000
        lgpio.tx_servo(self.gpio_handle, SERVO_PIN, width, freq)

    def manip_callback(self, message: ValveManip) -> None:
        if message.active:
            if not self.curr_active:
                self.curr_active = True
                self.servo(message.pwm)
        elif self.curr_active:
            self.curr_active = False
            self.servo(ValveManip.NEUTRAL_PWM)


def main() -> None:
    rclpy.init()

    subscriber = ValveManipulator()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
