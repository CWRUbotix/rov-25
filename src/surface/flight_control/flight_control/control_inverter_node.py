import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from rov_msgs.msg import CameraControllerSwitch, PixhawkInstruction

# Joystick curve
JOYSTICK_EXPONENT = 3


def joystick_map(raw: float) -> float:
    mapped = abs(raw) ** JOYSTICK_EXPONENT
    if raw < 0:
        mapped *= -1
    return mapped


class ControlInverterNode(Node):
    def __init__(self) -> None:
        super().__init__('control_inverter_node',
                         parameter_overrides=[])

        self.inverted = False

        self.inversion_subscription = self.create_subscription(
            CameraControllerSwitch,
            'camera_switch',
            self.invert_callback,
            QoSPresetProfiles.DEFAULT.value
        )

        self.control_subscription = self.create_subscription(
            PixhawkInstruction,
            'uninverted_pixhawk_control',
            self.control_callback,
            QoSPresetProfiles.DEFAULT.value
        )

        self.pixhawk_control = self.create_publisher(
            PixhawkInstruction,
            'pixhawk_control',
            QoSPresetProfiles.DEFAULT.value
        )

    def invert_callback(self, _: CameraControllerSwitch) -> None:
        self.inverted = not self.inverted

    def control_callback(self, msg: PixhawkInstruction) -> None:
        if self.inverted:
            msg.forward *= -1
            msg.lateral *= -1
            msg.pitch *= -1
            msg.roll *= -1

        self.pixhawk_control.publish(msg)


def main() -> None:
    rclpy.init()
    control_invert = ControlInverterNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(control_invert, executor=executor)
