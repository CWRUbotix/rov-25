from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING, Final

import rclpy
from mavros_msgs.srv import CommandBool, CommandLong
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default, QoSPresetProfiles
from sensor_msgs.msg import Joy

from mavros_msgs.msg import ManualControl, CommandCode
from rov_msgs.msg import CameraControllerSwitch, Manip, ValveManip
from flight_control.manual_control_utils import (
    apply_function,
    manual_control_to_tuple,
    tuple_to_manual_control,
)

if TYPE_CHECKING:
    from collections.abc import MutableSequence


UNPRESSED = 0
PRESSED = 1

# Button meanings for PS5 Control might be different for others
X_BUTTON = 0  # Manipulator 0
O_BUTTON = 1  # Manipulator 1
TRI_BUTTON = 2  # Manipulator 2
SQUARE_BUTTON = 3  # Manipulator 3
L1 = 4
R1 = 5
L2 = 6
R2 = 7
PAIRING_BUTTON = 8
MENU = 9
PS_BUTTON = 10
LJOYPRESS = 11
RJOYPRESS = 12
# Joystick Directions 1 is up/left -1 is down/right
# X is forward/backward Y is left/right
# L2 and R2 1 is not pressed and -1 is pressed
LJOYX = 0
LJOYY = 1
L2PRESS_PERCENT = 2
RJOYX = 3
RJOYY = 4
R2PRESS_PERCENT = 5
DPADHOR = 6
DPADVERT = 7

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
INSTR_EPSILON: Final = 0.05

ARMING_SERVICE_TIMEOUT = 3.0
ARM_MESSAGE = CommandBool.Request(value=True)
DISARM_MESSAGE = CommandBool.Request(value=False)

CONTROLLER_MODE_PARAM = 'controller_mode'
CONTROLLER_PROFILE_PARAM = 'controller_profile'


class ControllerMode(IntEnum):
    ARM = 0
    TOGGLE_CAMERAS = 1


@dataclass
class ControllerProfile:
    manip_left: int = L1
    manip_right: int = R1
    valve_clockwise: int = TRI_BUTTON
    valve_counterclockwise: int = SQUARE_BUTTON
    roll_left: int = X_BUTTON  # positive roll
    roll_right: int = O_BUTTON  # negative roll
    cam_toggle_left: int = PAIRING_BUTTON
    cam_toggle_right: int = MENU
    arm_button: int = MENU
    disarm_button: int = PAIRING_BUTTON
    lateral: int = LJOYX
    forward: int = LJOYY
    vertical_down: int = L2PRESS_PERCENT  # negative vertical value
    vertical_up: int = R2PRESS_PERCENT  # positive vertical value
    yaw: int = RJOYX
    pitch: int = RJOYY


CONTROLLER_PROFILES = (
    ControllerProfile(),
    ControllerProfile(
        manip_left=X_BUTTON,
        manip_right=O_BUTTON,
        roll_left=L1,
        roll_right=R1,
    ),
)

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
    smoothed_value = PREV_INSTR_FRAC * prev_value + NEXT_INSTR_FRAC * next_value

    # If close to target value, snap to it
    # (we want to get there eventually, not approach in the limit)
    if next_value - INSTR_EPSILON <= smoothed_value <= next_value + INSTR_EPSILON:
        return next_value

    return smoothed_value


def to_command_long(msg: ValveManip) -> CommandLong.Request:
    cl_msg = CommandLong.Request()

    cl_msg.command = CommandCode.DO_SET_SERVO
    cl_msg.confirmation = 0
    cl_msg.param1 = float(9)
    cl_msg.param2 = float(msg.pwm)

    return cl_msg

class ManualControlNode(Node):
    def __init__(self) -> None:
        super().__init__('manual_control_node')

        mode_param = self.declare_parameter(CONTROLLER_MODE_PARAM, value=ControllerMode.ARM)
        profile_param = self.declare_parameter(CONTROLLER_PROFILE_PARAM, value=0)
        self.profile = CONTROLLER_PROFILES[profile_param.value]

        self.inverted = False

        self.valve_manip = ValveManip(active=True, pwm=ValveManip.NEUTRAL_PWM)

        self.inversion_subscription = self.create_subscription(
            CameraControllerSwitch,
            'camera_switch',
            self.invert_callback,
            QoSPresetProfiles.DEFAULT.value,
        )

        self.subscription = self.create_subscription(
            Joy, 'joy', self.controller_callback, qos_profile_sensor_data
        )

        # Manipulators
        self.manip_publisher = self.create_publisher(
            Manip, 'manipulator_control', qos_profile_system_default
        )

        # Manual Control
        self.mc_pub = self.create_publisher(
            ManualControl, 'mavros/manual_control/send', QoSPresetProfiles.DEFAULT.value
        )

        # Valve Manip
        self.valve_cmd_client = self.create_client(CommandLong, 'mavros/cmd/command')

        self.previous_instruction_tuple: tuple[float, ...] = manual_control_to_tuple(
            ManualControl()
        )

        controller_mode = ControllerMode(mode_param.value)

        if controller_mode is ControllerMode.TOGGLE_CAMERAS:
            # Control camera switching
            self.misc_controls_callback = self.toggle_cameras
            self.camera_toggle_publisher = self.create_publisher(
                CameraControllerSwitch, 'camera_switch', qos_profile_system_default
            )
        else:
            self.misc_controls_callback = self.set_arming
            # Control arming
            self.arm_client = self.create_client(CommandBool, 'mavros/cmd/arming')

        self.manip_buttons: dict[int, ManipButton] = {
            self.profile.manip_left: ManipButton('left'),
            self.profile.manip_right: ManipButton('right'),
        }

        self.seen_left_cam = False
        self.seen_right_cam = False
        self.valve_manip_state = False

    def controller_callback(self, msg: Joy) -> None:
        self.joystick_to_manual_control(msg)
        self.valve_manip_callback(msg)
        self.manip_callback(msg)
        self.misc_controls_callback(msg)

    def joystick_to_manual_control(self, msg: Joy) -> None:
        axes: MutableSequence[float] = msg.axes
        buttons: MutableSequence[int] = msg.buttons

        mc_msg = ManualControl()

        mc_msg.x = float(axes[self.profile.forward])
        mc_msg.y = -float(axes[self.profile.lateral])
        mc_msg.z = float(axes[self.profile.vertical_down] - axes[self.profile.vertical_up]) / 2
        mc_msg.r = -float(axes[self.profile.yaw]) # Yaw
        mc_msg.enabled_extensions = EXTENSIONS_CODE
        mc_msg.s = float(axes[self.profile.pitch]) # Pitch
        mc_msg.t = float(buttons[self.profile.roll_left] - buttons[self.profile.roll_right]) # Roll

        mc_msg = apply_function(mc_msg, manual_control_map)
        mc_msg = self.smoothed_manual_control(mc_msg)
        mc_msg = self.invert_manual_control(mc_msg)
        
        self.mc_pub.publish(mc_msg)

    def manip_callback(self, msg: Joy) -> None:
        buttons: MutableSequence[int] = msg.buttons
        for button_id, manip_button in self.manip_buttons.items():
            just_pressed = buttons[button_id] == PRESSED
            if manip_button.last_button_state is False and just_pressed:
                new_manip_state = not manip_button.is_active
                manip_button.is_active = new_manip_state
                manip_msg = Manip(manip_id=manip_button.claw, activated=manip_button.is_active)
                self.manip_publisher.publish(manip_msg)

            manip_button.last_button_state = just_pressed

    def valve_manip_callback(self, msg: Joy) -> None:
        clockwise_pressed = msg.buttons[self.profile.valve_clockwise] == PRESSED
        counter_clockwise_pressed = msg.buttons[self.profile.valve_counterclockwise] == PRESSED

        if clockwise_pressed and not self.valve_manip_state:
            self.valve_manip = ValveManip(active=True, pwm=ValveManip.MAX_PWM)
            self.valve_manip_state = True
        elif counter_clockwise_pressed and not self.valve_manip_state:
            self.valve_manip = ValveManip(active=True, pwm=ValveManip.MIN_PWM)
            self.valve_manip_state = True
        elif self.valve_manip_state and not clockwise_pressed and not counter_clockwise_pressed:
            self.valve_manip = ValveManip(active=True, pwm=ValveManip.NEUTRAL_PWM)
            self.valve_manip_state = False

        self.valve_cmd_client.call_async(to_command_long(self.valve_manip))

    def toggle_cameras(self, msg: Joy) -> None:
        """Cycles through connected cameras on pilot GUI using menu and pairing buttons."""
        buttons: MutableSequence[int] = msg.buttons

        if buttons[self.profile.cam_toggle_right] == PRESSED:
            self.seen_right_cam = True
        elif buttons[self.profile.cam_toggle_left] == PRESSED:
            self.seen_left_cam = True
        elif buttons[self.profile.cam_toggle_right] == UNPRESSED and self.seen_right_cam:
            self.seen_right_cam = False
            self.camera_toggle_publisher.publish(CameraControllerSwitch(toggle_right=True))
        elif buttons[self.profile.cam_toggle_left] == UNPRESSED and self.seen_left_cam:
            self.seen_left_cam = False
            self.camera_toggle_publisher.publish(CameraControllerSwitch(toggle_right=False))

    def set_arming(self, msg: Joy) -> None:
        """Set the arming state using the menu and pairing buttons."""
        buttons: MutableSequence[int] = msg.buttons

        if buttons[self.profile.arm_button] == PRESSED:
            self.arm_client.call_async(ARM_MESSAGE)
        elif buttons[self.profile.disarm_button] == PRESSED:
            self.arm_client.call_async(DISARM_MESSAGE)

    def smooth_manual_control(self, msg: ManualControl) -> ManualControl:
        instruction_tuple = manual_control_to_tuple(msg)

        smoothed_tuple = tuple(
            smooth_value(previous_value, value)
            for (previous_value, value) in zip(
                self.previous_instruction_tuple, instruction_tuple, strict=True
            )
        )
        smoothed_instruction = tuple_to_manual_control(smoothed_tuple)

        self.previous_instruction_tuple = smoothed_tuple

        return smoothed_instruction

    def smoothed_manual_control(self, msg: ManualControl) -> ManualControl:
        msg = apply_function(msg, joystick_map)
        smoothed_instruction = self.smooth_manual_control(msg)
        return smoothed_instruction
    
    def invert_manual_control(self, msg: ManualControl) -> ManualControl:
        if self.inverted:
            msg.x *= -1 # Forward
            msg.y *= -1 # Lateral
            msg.s *= -1 # Pitch
            msg.t *= -1 # Roll

        return msg

    def invert_callback(self, _: CameraControllerSwitch) -> None:
        self.inverted = not self.inverted


class ManipButton:
    def __init__(self, claw: str) -> None:
        self.claw = claw
        self.last_button_state: bool = False
        self.is_active: bool = False


def main() -> None:
    rclpy.init()
    manual_control = ManualControlNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(manual_control, executor=executor)
    