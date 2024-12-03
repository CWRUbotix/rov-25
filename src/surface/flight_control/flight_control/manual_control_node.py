from enum import IntEnum
from typing import TYPE_CHECKING
from dataclasses import dataclass

import rclpy
from mavros_msgs.srv import CommandBool
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import Joy

from rov_msgs.msg import CameraControllerSwitch, Manip, PixhawkInstruction, ValveManip

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

ARMING_SERVICE_TIMEOUT = 3.0
ARM_MESSAGE = CommandBool.Request(value=True)
DISARM_MESSAGE = CommandBool.Request(value=False)

CONTROLLER_MODE_PARAM = 'controller_mode'


class ControllerMode(IntEnum):
    ARM = 0
    TOGGLE_CAMERAS = 1

@dataclass
class ControllerProfile:
    manip_left: int = X_BUTTON
    manip_right: int = O_BUTTON
    valve_clockwise: int = TRI_BUTTON
    valve_counterclockwise: int = SQUARE_BUTTON
    roll_left: int = L1
    roll_right: int = R1
    cam_toggle_left: int = PAIRING_BUTTON
    cam_toggle_right: int = MENU
    arm_button: int = MENU
    disarm_button: int = PAIRING_BUTTON
    lateral: int = LJOYX
    forward: int = LJOYY
    vertical_up: int = L2PRESS_PERCENT
    vertical_down: int = R2PRESS_PERCENT
    yaw: int = RJOYX
    pitch: int = RJOYY

CONTROLLER_PROFILES = [
    ControllerProfile(),
    ControllerProfile(manip_left=L1, manip_right=R1, roll_left=TRI_BUTTON, roll_right=SQUARE_BUTTON),
]

class ManualControlNode(Node):
    def __init__(self) -> None:
        super().__init__('manual_control_node')

        mode_param = self.declare_parameter(CONTROLLER_MODE_PARAM, value=ControllerMode.ARM)
        profile_param = self.declare_parameter('controller_profile', value=0)
        self.profile = CONTROLLER_PROFILES[profile_param.value]

        self.rc_pub = self.create_publisher(
            PixhawkInstruction, 'uninverted_pixhawk_control', qos_profile_system_default
        )

        self.subscription = self.create_subscription(
            Joy, 'joy', self.controller_callback, qos_profile_sensor_data
        )

        # Manipulators
        self.manip_publisher = self.create_publisher(
            Manip, 'manipulator_control', qos_profile_system_default
        )

        # Valve Manip
        self.valve_manip = self.create_publisher(
            ValveManip, 'valve_manipulator', qos_profile_system_default
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
        # if profile == ControllerProfile.PROFILE_0:
        #     self.manip_buttons: dict[int, ManipButton] = {
        #         X_BUTTON: ManipButton('left'),
        #         O_BUTTON: ManipButton('right'),
        #     }
        # elif profile == ControllerProfile.PROFILE_1:
        #     self.manip_buttons: dict[int, ManipButton] = {
        #         L1: ManipButton('left'),
        #         R1: ManipButton('right'),
        #     }


        self.seen_left_cam = False
        self.seen_right_cam = False
        self.valve_manip_state = False

    def controller_callback(self, msg: Joy) -> None:
        self.joystick_to_pixhawk(msg)
        self.valve_manip_callback(msg)
        self.manip_callback(msg)
        self.misc_controls_callback(msg)

    def joystick_to_pixhawk(self, msg: Joy) -> None:
        axes: MutableSequence[float] = msg.axes
        buttons: MutableSequence[int] = msg.buttons

        instruction = PixhawkInstruction(
            forward=float(axes[self.profile.forward]),
            lateral=-float(axes[self.profile.lateral]),
            vertical=float(axes[self.profile.vertical_up] - axes[self.profile.vertical_down]) / 2,
            roll=float(buttons[self.profile.roll_left] - buttons[self.profile.roll_right]),
            pitch=float(axes[self.profile.pitch]),
            yaw=-float(axes[self.profile.yaw]),
            author=PixhawkInstruction.MANUAL_CONTROL,
        )

        # if self.profile == ControllerProfile.PROFILE_0:
        #     instruction = PixhawkInstruction(
        #         forward=float(axes[LJOYY]),  # Left Joystick Y
        #         lateral=-float(axes[LJOYX]),  # Left Joystick X
        #         vertical=float(axes[L2PRESS_PERCENT] - axes[R2PRESS_PERCENT]) / 2,  # L2/R2 triggers
        #         roll=float(buttons[L1] - buttons[R1]),  # L1/R1 buttons
        #         pitch=float(axes[RJOYY]),  # Right Joystick Y
        #         yaw=-float(axes[RJOYX]),  # Right Joystick X
        #         author=PixhawkInstruction.MANUAL_CONTROL,
        #     )
        # elif self.profile == ControllerProfile.PROFILE_1:
        #     instruction = PixhawkInstruction(
        #         forward=float(axes[LJOYY]),  # Left Joystick Y
        #         lateral=-float(axes[LJOYX]),  # Left Joystick X
        #         vertical=float(axes[L2PRESS_PERCENT] - axes[R2PRESS_PERCENT]) / 2,  # L2/R2 triggers
        #         roll=float(buttons[X_BUTTON] - buttons[O_BUTTON]),  # L1/R1 buttons
        #         pitch=float(axes[RJOYY]),  # Right Joystick Y
        #         yaw=-float(axes[RJOYX]),  # Right Joystick X
        #         author=PixhawkInstruction.MANUAL_CONTROL,
        #     )

        self.rc_pub.publish(instruction)

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
        # if self.profile == ControllerProfile.PROFILE_0:
        #     clockwise_pressed = msg.buttons[TRI_BUTTON] == PRESSED
        #     counter_clockwise_pressed = msg.buttons[SQUARE_BUTTON] == PRESSED
        # elif self.profile == ControllerProfile.PROFILE_1:
        #     clockwise_pressed = msg.buttons[SQUARE_BUTTON] == PRESSED
        #     counter_clockwise_pressed = msg.buttons[TRI_BUTTON] == PRESSED
        clockwise_pressed = msg.buttons[self.profile.valve_clockwise] == PRESSED
        counter_clockwise_pressed = msg.buttons[self.profile.valve_counterclockwise] == PRESSED

        if clockwise_pressed and not self.valve_manip_state:
            self.valve_manip.publish(ValveManip(active=True, pwm=ValveManip.MAX_PWM))
            self.valve_manip_state = True
        elif counter_clockwise_pressed_pressed and not self.valve_manip_state:
            self.valve_manip.publish(ValveManip(active=True, pwm=ValveManip.MIN_PWM))
            self.valve_manip_state = True
        elif self.valve_manip_state and not clockwise_pressed and not counter_clockwise_pressed:
            self.valve_manip.publish(ValveManip(active=False))
            self.valve_manip_state = False
        # tri_pressed = msg.buttons[TRI_BUTTON] == PRESSED
        # square_pressed = msg.buttons[SQUARE_BUTTON] == PRESSED
        # if tri_pressed and not self.valve_manip_state:
        #     self.valve_manip.publish(ValveManip(active=True, pwm=ValveManip.MAX_PWM))
        #     self.valve_manip_state = True
        # elif square_pressed and not self.valve_manip_state:
        #     self.valve_manip.publish(ValveManip(active=True, pwm=ValveManip.MIN_PWM))
        #     self.valve_manip_state = True
        # elif self.valve_manip_state and not tri_pressed and not square_pressed:
        #     self.valve_manip.publish(ValveManip(active=False))
        #     self.valve_manip_state = False

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

        # if buttons[MENU] == PRESSED:
        #     self.seen_right_cam = True
        # elif buttons[PAIRING_BUTTON] == PRESSED:
        #     self.seen_left_cam = True
        # elif buttons[MENU] == UNPRESSED and self.seen_right_cam:
        #     self.seen_right_cam = False
        #     self.camera_toggle_publisher.publish(CameraControllerSwitch(toggle_right=True))
        # elif buttons[PAIRING_BUTTON] == UNPRESSED and self.seen_left_cam:
        #     self.seen_left_cam = False
        #     self.camera_toggle_publisher.publish(CameraControllerSwitch(toggle_right=False))

    def set_arming(self, msg: Joy) -> None:
        """Set the arming state using the menu and pairing buttons."""
        buttons: MutableSequence[int] = msg.buttons

        if buttons[self.profile.arm_button] == PRESSED:
            self.arm_client.call_async(ARM_MESSAGE)
        elif buttons[self.profile.disarm_button] == PRESSED:
            self.arm_client.call_async(DISARM_MESSAGE)

        # if buttons[MENU] == PRESSED:
        #     self.arm_client.call_async(ARM_MESSAGE)
        # elif buttons[PAIRING_BUTTON] == PRESSED:
        #     self.arm_client.call_async(DISARM_MESSAGE)


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
