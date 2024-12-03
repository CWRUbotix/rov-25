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
SPEED_THROTTLE: Final = 0.65

# Joystick curve
JOYSTICK_EXPONENT: Final = 3

# Range of values Pixhawk takes
# In microseconds
ZERO_SPEED: Final = 0
Z_ZERO_SPEED: Final = 500
MAX_RANGE_SPEED: Final = 2000
Z_MAX_RANGE_SPEED: Final = 1000
RANGE_SPEED: Final = MAX_RANGE_SPEED * SPEED_THROTTLE
Z_RANGE_SPEED: Final = Z_MAX_RANGE_SPEED * SPEED_THROTTLE

EXTENSIONS_CODE: Final = 0b00000011

NEXT_INSTR_FRAC: Final = 0.05
PREV_INSTR_FRAC: Final = 1 - NEXT_INSTR_FRAC


def joystick_map(raw: float) -> float:
    """
    Convert the provided joystick position to a
    float in [-1.0, 1.0] for use in a PixhawkInstruction.

    Parameters
    ----------
    raw : float
        The joystick position to convert

    Returns
    -------
    float
        A float in [-1.0, 1.0] to act as a PixhawkInstruction dimension
    """
    mapped = abs(raw) ** JOYSTICK_EXPONENT
    if raw < 0:
        mapped *= -1
    return mapped


def manual_control_map(value: float) -> float:
    """
    Convert the provided float in [-1.0, 1.0] to a ManualControl dimension.

    Parameters
    ----------
    value : float
        The float in [-1.0, 1.0] to convert to a ManualControl dimension

    Returns
    -------
    float
        The resulting ManualControl dimension
    """
    return RANGE_SPEED * value + ZERO_SPEED

def smooth_value(prev_value: float, next_value: float) -> float:
    """
    Get a value that interpolates prev_value & next_value.

    Parameters
    ----------
    prev_value : float
        The previous value, affects the result based on PREV_INSTR_FRAC
    next_value : float
        The next value, affects the result based on NEXT_INSTR_FRAC

    Returns
    -------
    float
        The resulting value between prev_value & next_value
    """
    return PREV_INSTR_FRAC * prev_value + NEXT_INSTR_FRAC * next_value

def to_manual_control(msg: PixhawkInstruction) -> ManualControl:
    """
    Convert the provided PixhawkInstruction to a ManualControl message.

    Parameters
    ----------
    msg : PixhawkInstruction
        The PixhawkInstruction to convert

    Returns
    -------
    ManualControl
        The resulting ManualControl message
    """
    mc_msg = ManualControl()

    # Maps to PWM
    mapped_msg = apply_function(msg, manual_control_map)

    # To account for different z limits
    mapped_msg.vertical = Z_RANGE_SPEED * msg.vertical + Z_ZERO_SPEED

    mc_msg.x = mapped_msg.forward
    mc_msg.z = mapped_msg.vertical
    mc_msg.y = mapped_msg.lateral
    mc_msg.r = mapped_msg.yaw
    mc_msg.enabled_extensions = EXTENSIONS_CODE
    mc_msg.s = mapped_msg.pitch
    mc_msg.t = mapped_msg.roll

    return mc_msg


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

        self.previous_instruction_tuple: tuple[float, ...] = pixhawk_instruction_to_tuple(
            PixhawkInstruction()
        )

    def smooth_pixhawk_instruction(self, msg: PixhawkInstruction) -> PixhawkInstruction:
        instruction_tuple = pixhawk_instruction_to_tuple(msg)

        smoothed_tuple = tuple(
            smooth_value(previous_value, value)
            for (previous_value, value) in zip(
                self.previous_instruction_tuple, instruction_tuple, strict=True
            )
        )
        smoothed_instruction = tuple_to_pixhawk_instruction(smoothed_tuple, msg.author)

        self.previous_instruction_tuple = smoothed_tuple

        return smoothed_instruction

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
            msg = apply_function(msg, joystick_map)
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

        smoothed_instruction = self.smooth_pixhawk_instruction(msg)
        self.mc_pub.publish(to_manual_control(smoothed_instruction))


def main() -> None:
    rclpy.init()
    control_invert = MultiplexerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(control_invert, executor=executor)
