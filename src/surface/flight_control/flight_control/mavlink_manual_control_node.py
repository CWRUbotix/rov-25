from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING

import rclpy
from mavros_msgs.srv import CommandBool
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import Joy

from rov_msgs.msg import CameraControllerSwitch, Manip, PixhawkInstruction, ValveManip

if TYPE_CHECKING:
    from collections.abc import MutableSequence

from pymavlink import mavutil


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


class MavlinkManualControlNode(Node):
    def __init__(self) -> None:
        super().__init__('manual_control_node')

        mode_param = self.declare_parameter(CONTROLLER_MODE_PARAM, value=ControllerMode.ARM)
        profile_param = self.declare_parameter(CONTROLLER_PROFILE_PARAM, value=0)
        self.profile = CONTROLLER_PROFILES[profile_param.value]

        self.subscription = self.create_subscription(
            Joy, 'joy', self.controller_callback, qos_profile_sensor_data
        )

        # # Manipulators
        # self.manip_publisher = self.create_publisher(
        #     Manip, 'manipulator_control', qos_profile_system_default
        # )

        # # Valve Manip
        # self.valve_manip = self.create_publisher(
        #     ValveManip, 'valve_manipulator', qos_profile_system_default
        # )

        # controller_mode = ControllerMode(mode_param.value)

        # if controller_mode is ControllerMode.TOGGLE_CAMERAS:
        #     # Control camera switching
        #     self.misc_controls_callback = self.toggle_cameras
        #     self.camera_toggle_publisher = self.create_publisher(
        #         CameraControllerSwitch, 'camera_switch', qos_profile_system_default
        #     )
        # else:
        #     self.misc_controls_callback = self.set_arming
        #     # Control arming
        #     self.arm_client = self.create_client(CommandBool, 'mavros/cmd/arming')

        # self.manip_buttons: dict[int, ManipButton] = {
        #     self.profile.manip_left: ManipButton('left'),
        #     self.profile.manip_right: ManipButton('right'),
        # }

        # self.seen_left_cam = False
        # self.seen_right_cam = False
        # self.valve_manip_state = False

        self.mavlink = mavutil.mavlink_connection('udpin:0.0.0.0:14550', source_system=255)
        self.mavlink.wait_heartbeat()
        self.get_logger().info("Mavlink connected!")

        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        self.mavlink.motors_armed_wait()
        self.get_logger().info("Vehicle armed")


    def controller_callback(self, msg: Joy) -> None:
        self.send_mavlink_control(msg)
        
        # self.valve_manip_callback(msg)
        # self.manip_callback(msg)
        # self.misc_controls_callback(msg)

    def send_mavlink_control(self, msg: Joy) -> None:
        mavlink_msg =self.mavlink.recv_match()
        if mavlink_msg:
            if mavlink_msg.get_type() == 'HEARTBEAT':
                # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
                self.get_logger().info(f"sys: {mavlink_msg._header.srcSystem}, comp: {mavlink_msg._header.srcComponent}, system status: {mavlink_msg.system_status}")
        
        axes: MutableSequence[float] = msg.axes
        buttons: MutableSequence[int] = msg.buttons

        self.mavlink.mav.manual_control_send(
            self.mavlink.target_system,
            int(axes[self.profile.forward] * 1000),
            int(-axes[self.profile.lateral] * 1000),
            int((axes[self.profile.vertical_down] - axes[self.profile.vertical_up] + 1) / 2 * 1000),
            int(axes[self.profile.yaw] * 1000),
            int(axes[self.profile.pitch] * 1000),
            int((buttons[self.profile.roll_left] - buttons[self.profile.roll_right]) * 1000),
        )

#     def manip_callback(self, msg: Joy) -> None:
#         buttons: MutableSequence[int] = msg.buttons
#         for button_id, manip_button in self.manip_buttons.items():
#             just_pressed = buttons[button_id] == PRESSED
#             if manip_button.last_button_state is False and just_pressed:
#                 new_manip_state = not manip_button.is_active
#                 manip_button.is_active = new_manip_state
#                 manip_msg = Manip(manip_id=manip_button.claw, activated=manip_button.is_active)
#                 self.manip_publisher.publish(manip_msg)

#             manip_button.last_button_state = just_pressed

#     def valve_manip_callback(self, msg: Joy) -> None:
#         clockwise_pressed = msg.buttons[self.profile.valve_clockwise] == PRESSED
#         counter_clockwise_pressed = msg.buttons[self.profile.valve_counterclockwise] == PRESSED

#         if clockwise_pressed and not self.valve_manip_state:
#             self.valve_manip.publish(ValveManip(active=True, pwm=ValveManip.MAX_PWM))
#             self.valve_manip_state = True
#         elif counter_clockwise_pressed and not self.valve_manip_state:
#             self.valve_manip.publish(ValveManip(active=True, pwm=ValveManip.MIN_PWM))
#             self.valve_manip_state = True
#         elif self.valve_manip_state and not clockwise_pressed and not counter_clockwise_pressed:
#             self.valve_manip.publish(ValveManip(active=False))
#             self.valve_manip_state = False

#     def toggle_cameras(self, msg: Joy) -> None:
#         """Cycles through connected cameras on pilot GUI using menu and pairing buttons."""
#         buttons: MutableSequence[int] = msg.buttons

#         if buttons[self.profile.cam_toggle_right] == PRESSED:
#             self.seen_right_cam = True
#         elif buttons[self.profile.cam_toggle_left] == PRESSED:
#             self.seen_left_cam = True
#         elif buttons[self.profile.cam_toggle_right] == UNPRESSED and self.seen_right_cam:
#             self.seen_right_cam = False
#             self.camera_toggle_publisher.publish(CameraControllerSwitch(toggle_right=True))
#         elif buttons[self.profile.cam_toggle_left] == UNPRESSED and self.seen_left_cam:
#             self.seen_left_cam = False
#             self.camera_toggle_publisher.publish(CameraControllerSwitch(toggle_right=False))

#     def set_arming(self, msg: Joy) -> None:
#         """Set the arming state using the menu and pairing buttons."""
#         buttons: MutableSequence[int] = msg.buttons

#         if buttons[self.profile.arm_button] == PRESSED:
#             self.arm_client.call_async(ARM_MESSAGE)
#         elif buttons[self.profile.disarm_button] == PRESSED:
#             self.arm_client.call_async(DISARM_MESSAGE)


# class ManipButton:
#     def __init__(self, claw: str) -> None:
#         self.claw = claw
#         self.last_button_state: bool = False
#         self.is_active: bool = False


def main() -> None:
    rclpy.init()
    manual_control = MavlinkManualControlNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(manual_control, executor=executor)
