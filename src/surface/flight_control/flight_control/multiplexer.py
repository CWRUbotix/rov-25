from collections.abc import Callable
from typing import Final

import rclpy
from mavros_msgs.msg import ManualControl
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from rov_msgs.msg import PixhawkInstruction
from rov_msgs.srv import AutonomousFlight

# Brown out protection
SPEED_THROTTLE = 0.85

# Joystick curve
JOYSTICK_EXPONENT = 3

# Range of values Pixhawk takes
# In microseconds
ZERO_SPEED: Final = 0
Z_ZERO_SPEED: Final = 500
MAX_RANGE_SPEED: Final = 2000
Z_MAX_RANGE_SPEED: Final = 1000
RANGE_SPEED: Final = MAX_RANGE_SPEED * SPEED_THROTTLE
Z_RANGE_SPEED: Final = Z_MAX_RANGE_SPEED * SPEED_THROTTLE

EXTENSIONS_CODE: Final = 0b00000011

# Channels for RC command
MAX_CHANNEL = 8
MIN_CHANNEL = 1

FORWARD_CHANNEL = 4  # X
THROTTLE_CHANNEL = 2  # Z (vertical)
LATERAL_CHANNEL = 5  # Y (left & right)
PITCH_CHANNEL = 0  # Pitch
YAW_CHANNEL = 3  # Yaw
ROLL_CHANNEL = 1  # Roll


def joystick_map(raw: float) -> float:
    mapped = abs(raw) ** JOYSTICK_EXPONENT
    if raw < 0:
        mapped *= -1
    return mapped


class MultiplexerNode(Node):
    def __init__(self) -> None:
        super().__init__('multiplexer', parameter_overrides=[])

        self.state = AutonomousFlight.Request.STOP

        self.autonomous_toggle = self.create_service(
            AutonomousFlight, 'auto_control_toggle', self.state_control
        )

        self.control_subscription = self.create_subscription(
            PixhawkInstruction,
            'pixhawk_control',
            self.control_callback,
            QoSPresetProfiles.DEFAULT.value,
        )

        self.mc_pub = self.create_publisher(
            ManualControl, 'mavros/manual_control/send', QoSPresetProfiles.DEFAULT.value
        )

    @staticmethod
    def apply(msg: PixhawkInstruction, function_to_apply: Callable[[float], float]) -> None:
        """Apply a function to each dimension of this PixhawkInstruction."""
        msg.forward = function_to_apply(msg.forward)
        msg.vertical = msg.vertical
        msg.lateral = function_to_apply(msg.lateral)
        msg.pitch = function_to_apply(msg.pitch)
        msg.yaw = function_to_apply(msg.yaw)
        msg.roll = function_to_apply(msg.roll)

    @staticmethod
    def to_manual_control(msg: PixhawkInstruction) -> ManualControl:
        """Convert this PixhawkInstruction to an rc_msg with the appropriate channels array."""
        mc_msg = ManualControl()

        # Maps to PWM
        MultiplexerNode.apply(msg, lambda value: (RANGE_SPEED * value) + ZERO_SPEED)

        mc_msg.x = msg.forward
        mc_msg.z = (
            Z_RANGE_SPEED * msg.vertical
        ) + Z_ZERO_SPEED  # To account for different z limits
        mc_msg.y = msg.lateral
        mc_msg.r = msg.yaw
        mc_msg.enabled_extensions = EXTENSIONS_CODE
        mc_msg.s = msg.pitch
        mc_msg.t = msg.roll

        return mc_msg

    def state_control(
        self, req: AutonomousFlight.Request, res: AutonomousFlight.Response
    ) -> AutonomousFlight.Response:
        self.state = req.state
        res.current_state = req.state
        return res

    def control_callback(self, msg: PixhawkInstruction) -> None:
        if (
            msg.author == PixhawkInstruction.MANUAL_CONTROL
            and self.state == AutonomousFlight.Request.STOP
        ):
            # Smooth out adjustments
            # TODO: look into maybe doing inheritance on a PixhawkInstruction
            MultiplexerNode.apply(msg, joystick_map)
        elif (
            msg.author == PixhawkInstruction.KEYBOARD_CONTROL
            and self.state == AutonomousFlight.Request.STOP
        ) or (
            msg.author == PixhawkInstruction.AUTONOMOUS_CONTROL
            and self.state == AutonomousFlight.Request.START
        ):
            pass
        else:
            return

        self.mc_pub.publish(self.to_manual_control(msg))


def main() -> None:
    rclpy.init()
    control_invert = MultiplexerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(control_invert, executor=executor)
