from typing import Final

import rclpy
from mavros_msgs.msg import ManualControl
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from flight_control.pixhawk_instruction_utils import (
    apply_function,
    pixhawk_instruction_to_tuple,
    tuple_to_pixhawk_instruction,
)
from rov_msgs.msg import PixhawkInstruction
from rov_msgs.srv import AutonomousFlight

# Brown out protection
SPEED_THROTTLE = 0.65

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

NEXT_INSTR_FRAC: Final[float] = 0.05
PREV_INSTR_FRAC: Final[float] = 1 - NEXT_INSTR_FRAC


def joystick_map(raw: float) -> float:
    mapped = abs(raw) ** JOYSTICK_EXPONENT
    if raw < 0:
        mapped *= -1
    return mapped


def manual_control_map(value: float) -> float:
    return RANGE_SPEED * value + ZERO_SPEED


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

        self.previous_instruction = PixhawkInstruction(author=PixhawkInstruction.MANUAL_CONTROL)

    @staticmethod
    def smooth_value(prev_value: float, next_value: float) -> float:
        return PREV_INSTR_FRAC * prev_value + NEXT_INSTR_FRAC * next_value

    def smooth_pixhawk_instruction(self, msg: PixhawkInstruction) -> PixhawkInstruction:
        instruction_tuple = pixhawk_instruction_to_tuple(msg)
        previous_instruction_tuple = pixhawk_instruction_to_tuple(self.previous_instruction)

        instruction_tuple = tuple(
            MultiplexerNode.smooth_value(previous_value, value)
            for (previous_value, value) in zip(
                previous_instruction_tuple, instruction_tuple, strict=True
            )
        )
        smoothed_instruction = tuple_to_pixhawk_instruction(instruction_tuple, msg.author)

        self.previous_instruction = smoothed_instruction

        return smoothed_instruction

    # def apply(msg: PixhawkInstruction, function_to_apply: Callable[[float], float]) -> None:
    #     """Apply a function to each dimension of this PixhawkInstruction."""
    #     msg.forward = function_to_apply(msg.forward)
    #     msg.vertical = msg.vertical
    #     msg.lateral = function_to_apply(msg.lateral)
    #     msg.pitch = function_to_apply(msg.pitch)
    #     msg.yaw = function_to_apply(msg.yaw)
    #     msg.roll = function_to_apply(msg.roll)

    @staticmethod
    def to_manual_control(msg: PixhawkInstruction) -> ManualControl:
        """Convert this PixhawkInstruction to an rc_msg with the appropriate channels array."""
        mc_msg = ManualControl()

        # Maps to PWM
        # instruction_tuple = pixhawk_instruction_to_tuple(msg)
        # instruction_tuple = tuple(manual_control_map(value) for value in instruction_tuple)
        # mapped_msg = tuple_to_pixhawk_instruction(instruction_tuple)
        mapped_msg = apply_function(msg, manual_control_map)

        # To account for different z limits
        mapped_msg.vertical = Z_RANGE_SPEED * msg.vertical + Z_ZERO_SPEED

        # MultiplexerNode.apply(msg, lambda value: (RANGE_SPEED * value) + ZERO_SPEED)

        mc_msg.x = mapped_msg.forward
        mc_msg.z = mapped_msg.vertical
        # (
        #     Z_RANGE_SPEED * mapped_msg.vertical
        # ) + Z_ZERO_SPEED  # To account for different z limits
        mc_msg.y = mapped_msg.lateral
        mc_msg.r = mapped_msg.yaw
        mc_msg.enabled_extensions = EXTENSIONS_CODE
        mc_msg.s = mapped_msg.pitch
        mc_msg.t = mapped_msg.roll

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
            # instruction_tuple = pixhawk_instruction_to_tuple(msg)
            # instruction_tuple = tuple(joystick_map(value) for value in instruction_tuple)
            # msg = tuple_to_pixhawk_instruction(instruction_tuple)
            msg = apply_function(msg, joystick_map)
        elif (
            msg.author == PixhawkInstruction.KEYBOARD_CONTROL
            and self.state == AutonomousFlight.Request.STOP
            or msg.author == PixhawkInstruction.AUTONOMOUS_CONTROL
            and self.state == AutonomousFlight.Request.START
        ):
            pass
        else:
            return

        smoothed_instruction = self.smooth_pixhawk_instruction(msg)
        self.mc_pub.publish(self.to_manual_control(smoothed_instruction))


def main() -> None:
    rclpy.init()
    control_invert = MultiplexerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(control_invert, executor=executor)
