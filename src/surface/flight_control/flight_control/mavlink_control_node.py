from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING
import time
import math
import os

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import Joy

from rov_msgs.msg import Manip, Heartbeat
from rov_msgs.msg import VehicleState as VehicleStateMsg
from rov_msgs.srv import VehicleArming


if TYPE_CHECKING:
    from collections.abc import MutableSequence

from pymavlink import mavutil


NATIVE_JOYSTICK = True
if NATIVE_JOYSTICK:
    from pygame import joystick
    os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"


JOY_MAP_STRENGTH = 2  # 1 disables joystick mapping, higher number decrease intermediate joystick values to allow for finer control
GLOBAL_THROTTLE = 1.0  # From 0 to 1, lower numbers decrease the thrust in every direction
PITCH_THROTTLE = 0.5  # From 0 to 1, stacks multiplicatively with GLOBAL_THROTTLE

MAVLINK_POLL_RATE = 20  # Hz
PI_TIMEOUT = 1  # Seconds since last heartbeat before the pi is considered disconnected
ARDUSUB_TIMEOUT = 3  # Seconds since last heartbeat before ardusub is considered disconnected

VEHICLE_COMPONENT_ID = 1
MANUAL_CONTROL_EXTENSIONS_CODE = 0b00000011


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

# # Button meanings for PS5 Control might be different for others
# X_BUTTON = 0  # Manipulator 0
# O_BUTTON = 1  # Manipulator 1
# TRI_BUTTON = 2  # Manipulator 2
# SQUARE_BUTTON = 3  # Manipulator 3
# L1 = 4
# R1 = 5
# L2 = 6
# R2 = 7
# PAIRING_BUTTON = 4
# MENU = 6
# PS_BUTTON = 10
# LJOYPRESS = 11
# RJOYPRESS = 12
# # Joystick Directions 1 is up/left -1 is down/right
# # X is forward/backward Y is left/right
# # L2 and R2 1 is not pressed and -1 is pressed
# LJOYX = 0
# LJOYY = 1
# RJOYX = 2
# RJOYY = 3
# L2PRESS_PERCENT = 4
# R2PRESS_PERCENT = 5
# DPADHOR = 6
# DPADVERT = 7

CONTROLLER_MODE_PARAM = 'controller_mode'
CONTROLLER_PROFILE_PARAM = 'controller_profile'


class ControllerMode(IntEnum):
    ARM = 0
    TOGGLE_CAMERAS = 1


class ManipButton:
    def __init__(self, claw: str) -> None:
        self.claw = claw
        self.last_button_state: bool = False
        self.is_active: bool = False


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


@dataclass
class VehicleState:
    pi_connected: bool = False
    ardusub_connected: bool = False
    armed: bool = False



class MavlinkManualControlNode(Node):
    def __init__(self) -> None:
        super().__init__('mavlink_control_node')

        mode_param = self.declare_parameter(CONTROLLER_MODE_PARAM, value=ControllerMode.ARM)
        profile_param = self.declare_parameter(CONTROLLER_PROFILE_PARAM, value=0)
        self.profile = CONTROLLER_PROFILES[profile_param.value]

        if not NATIVE_JOYSTICK:
            self.subscription = self.create_subscription(
                Joy, 'joy', self.controller_callback, qos_profile_sensor_data
            )

        # Manipulators
        self.manip_publisher = self.create_publisher(
            Manip, 'manipulator_control', qos_profile_system_default
        )

        self.manip_buttons: dict[int, ManipButton] = {
            self.profile.manip_left: ManipButton('left'),
            self.profile.manip_right: ManipButton('right'),
        }

        # os.environ["MAVLINK20"] = '1'
        self.get_logger().info("Connecting to mavlink...")
        self.mavlink = mavutil.mavlink_connection('udpin:0.0.0.0:14550', source_system=255)

        # Send mavlink arm message
        # self.mavlink.mav.command_long_send(
        #     self.mavlink.target_system,
        #     self.mavlink.target_component,
        #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        #     0,
        #     1, 0, 0, 0, 0, 0, 0
        # )
        # self.mavlink.motors_armed_wait()
        # self.get_logger().info("Vehicle armed")

        self.state_publisher = self.create_publisher(
            VehicleStateMsg, 'vehicle_state_event', qos_profile_system_default
        )

        self.mavros_subscription = self.create_subscription(
            Heartbeat, 'pi_heartbeat', self.pi_heartbeat_callback, 10
        )

        self.arming_service = self.create_service(
            VehicleArming, 'arming', callback=self.arming_service_callback
        )

        self.last_pi_heartbeat: float = 0  # Unix timestamp of the last mavlink heartbeat from the pi
        self.last_ardusub_heartbeat: float = 0  # Unix timestamp of the last mavlink heartbeat from the pi
        self.last_state_subscriber_count: int = 0
        
        self.vehicle_state = VehicleState()

        self.timer = self.create_timer(1 / MAVLINK_POLL_RATE, self.poll_mavlink)


    def controller_callback(self, msg: Joy) -> None:
        self.process_arming_buttons(msg)
        self.send_mavlink_control(msg)        
        self.manip_callback(msg)

    def joystick_map(self, raw: float) -> float:
        """Apply a mapping to a joystick axis before it is used for control

        Parameters
        ----------
        raw : float
            The position of the joystick, from -1 to 1

        Returns
        -------
        float
            The output of the mapping, between -1 and 1
        """
        return math.copysign(math.fabs(raw) ** JOY_MAP_STRENGTH, raw) * GLOBAL_THROTTLE

    def send_mavlink_control(self, msg: Joy) -> None:        
        axes: MutableSequence[float] = msg.axes

        # self.get_logger().info(str(axes))
        # self.get_logger().info(str((self.joystick_map(axes[self.profile.vertical_down] / 2 + 0.5) - self.joystick_map(axes[self.profile.vertical_up] / 2 + 0.5)) / 2 * 1000 + 500))

        self.mavlink.mav.manual_control_send(
            self.mavlink.target_system,
            int(self.joystick_map(axes[self.profile.forward]) * 1000),
            int(-self.joystick_map(axes[self.profile.lateral]) * 1000),
            int((self.joystick_map(axes[self.profile.vertical_down] / 2 + 0.5) - self.joystick_map(axes[self.profile.vertical_up] / 2 + 0.5)) / 2 * 1000 + 500),
            int(-self.joystick_map(axes[self.profile.yaw]) * 1000),
            0, 0,
            MANUAL_CONTROL_EXTENSIONS_CODE,
            int(self.joystick_map(axes[self.profile.pitch]) * PITCH_THROTTLE * 1000),
            0  #int((buttons[self.profile.roll_left] - buttons[self.profile.roll_right]) * 1000),
        )

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


    def set_armed(self, arm: bool) -> None:
        """Send a mavlink message to arm or disarm the vehicle

        Parameters
        ----------
        arm : bool
            Whether the vehicle should be armed (True) or disarmed (False)
        """
        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            int(arm),
            0, 0, 0, 0, 0, 0
        )
        self.get_logger().info(f"Setting armed state to {arm}")

    def arming_service_callback(self, request: VehicleArming.Request, response: VehicleArming.Response
                                ) -> VehicleArming.Response:
        """Handle a request to arm or disarm the robot

        Parameters
        ----------
        request : VehicleArming.Request
            The ROS service request that resulted in this callback being called
        response : VehicleArming.Response
            The ROS service response to be sent to the node that made the service call

        Returns
        -------
        VehicleArming.Response
            The filled ROS service response
        """

        self.set_armed(request.arm)

        response.message_sent = True
        return response


    def process_arming_buttons(self, msg: Joy) -> None:
        """Set the arming state using the menu and pairing buttons."""
        buttons: MutableSequence[int] = msg.buttons

        if buttons[self.profile.disarm_button] == PRESSED:
            self.get_logger().info("Sending disarm command")
            self.set_armed(False)
        elif buttons[self.profile.arm_button] == PRESSED:
            self.get_logger().info("Sending arm command")
            self.set_armed(True)

    def publish_state(self, state: VehicleState) -> None:
        self.state_publisher.publish(
            VehicleStateMsg(
                pi_connected=state.pi_connected,
                pixhawk_connected=state.ardusub_connected,
                armed=state.armed,
            )
        )

    def poll_mavlink(self) -> None:
        new_state = VehicleState(
            pi_connected=self.vehicle_state.pi_connected,
            ardusub_connected=self.vehicle_state.ardusub_connected,
            armed=self.vehicle_state.armed
        )

        mavlink_msg=self.mavlink.recv_match()
        while mavlink_msg:
            if mavlink_msg._header.srcComponent == VEHICLE_COMPONENT_ID and mavlink_msg.get_type() == 'HEARTBEAT':
                # self.get_logger().info(f"sys: {mavlink_msg._header.srcSystem}, comp: {mavlink_msg._header.srcComponent}, system status: {mavlink_msg.system_status}")

                self.last_ardusub_heartbeat = time.time()

                new_state.ardusub_connected = True
                if not self.vehicle_state.ardusub_connected:
                    self.get_logger().info("Ardusub connected")
                
                if mavlink_msg.system_status == 4:  # MAV_STATE_STANDBY
                    new_state.armed = True
                    if not self.vehicle_state.armed:
                        self.get_logger().info("Vehicle armed")
                elif mavlink_msg.system_status == 3:  # MAV_STATE_ACTIVE
                    new_state.armed = False
                    if self.vehicle_state.armed:
                        self.get_logger().info("Vehicle disarmed")
                else:
                    self.get_logger().warning(f'Unknown ardusub state: {mavlink_msg.system_status}')

            mavlink_msg=self.mavlink.recv_match()

        # Check pi timeeout
        if self.vehicle_state.pi_connected and time.time() - self.last_pi_heartbeat > PI_TIMEOUT:
            self.vehicle_state.pi_connected = False
            self.publish_state(self.vehicle_state)
            self.get_logger().warning('Pi disconnected')

        # Check ardusub timeeout
        if self.vehicle_state.ardusub_connected and time.time() - self.last_ardusub_heartbeat > ARDUSUB_TIMEOUT:
            new_state.ardusub_connected = False
            self.publish_state(self.vehicle_state)
            self.get_logger().warning('Ardusub disconnected')

        if self.vehicle_state != new_state:
            self.vehicle_state = new_state
            self.publish_state(new_state)
        
        # Check if any new nodes have subscribed to state events, and update them if so
        self.poll_subscribers()


    def pi_heartbeat_callback(self, _: Heartbeat) -> None:
        self.last_pi_heartbeat = time.time()

        if not self.vehicle_state.pi_connected:
            self.vehicle_state.pi_connected = True
            self.publish_state(self.vehicle_state)
            self.get_logger().info('Pi connected')


    def poll_subscribers(self) -> None:
        # Whenever a node subscribes to vehicle state updates, send the current state
        subscriber_count = self.state_publisher.get_subscription_count()
        if subscriber_count > self.last_state_subscriber_count:
            self.publish_state(self.vehicle_state)

        self.last_state_subscriber_count = subscriber_count


def main() -> None:
    rclpy.init()
    manual_control = MavlinkManualControlNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(manual_control, executor=executor)
