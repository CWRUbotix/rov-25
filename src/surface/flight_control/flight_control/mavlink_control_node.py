import math
import os
import time
from dataclasses import dataclass

import rclpy
import rclpy.utilities
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rov_msgs.msg import Heartbeat, Manip, VehicleState
from rov_msgs.srv import VehicleArming

os.environ['MAVLINK20'] = '1'  # Force mavlink 2.0 for pymavlink
from pymavlink import mavutil

os.environ['SDL_VIDEODRIVER'] = 'dummy'
os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
import pygame
from pygame import joystick

JOYSTICK_POLL_RATE = 50  # Hz

# A strength of 1 disables joystick mapping, higher numbers
# decrease intermediate joystick values to allow for finer control
JOY_MAP_STRENGTH = 2

GLOBAL_THROTTLE = 1.0  # From 0 to 1, lower numbers decrease the thrust in every direction
PITCH_THROTTLE = 0.5  # From 0 to 1, stacks multiplicatively with GLOBAL_THROTTLE

# How often to check for new subscribers and send them the current state, Hz
SUBSCRIBER_POLL_RATE = 2

# How frequently to check for mavlink heartbeats, Hz, cannot be greater than joystick polling rate
MAVLINK_POLL_RATE = 20

PI_TIMEOUT = 1  # Seconds since last heartbeat before the pi is considered disconnected
ARDUSUB_TIMEOUT = 3  # Seconds since last heartbeat before ardusub is considered disconnected

MAVLINK_CONNECTION_STRING = 'udpin:0.0.0.0:14550'
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
DPAD_LEFT = 13
DPAD_RIGHT = 14
DPAD_DOWN = 15
DPAD_UP = 16
# Joystick Directions 1 is up/left -1 is down/right
# X is forward/backward Y is left/right
# L2 and R2 1 is not pressed and -1 is pressed
LJOYX = 0
LJOYY = 1
L2PRESS_PERCENT = 2
RJOYX = 3
RJOYY = 4
R2PRESS_PERCENT = 5


CONTROLLER_PROFILE_PARAM = 'controller_profile'


@dataclass
class ManipButton:
    claw: str
    last_button_state: bool = False
    is_active: bool = False


@dataclass
class ControllerProfile:
    manip_left: int = L1
    manip_right: int = R1
    valve_clockwise: int = TRI_BUTTON
    valve_counterclockwise: int = SQUARE_BUTTON
    roll_left: int = X_BUTTON  # positive roll
    roll_right: int = O_BUTTON  # negative roll
    arm_button: int = MENU
    disarm_button: int = PAIRING_BUTTON
    cam_front_button: int = DPAD_UP
    cam_back_button: int = DPAD_DOWN
    lateral: int = LJOYX
    forward: int = LJOYY
    vertical_down: int = L2PRESS_PERCENT  # negative vertical value
    vertical_up: int = R2PRESS_PERCENT  # positive vertical value
    yaw: int = RJOYX
    pitch: int = RJOYY


@dataclass
class JoystickState:
    axes: list[float]
    buttons: list[bool]


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
        super().__init__('mavlink_control_node')

        profile_param = self.declare_parameter(CONTROLLER_PROFILE_PARAM, value=0)
        self.profile: ControllerProfile = CONTROLLER_PROFILES[profile_param.value]

        self.joysticks: dict[int, pygame.joystick.JoystickType] = {}
        self.current_joystick_id: int | None = None
        pygame.init()
        pygame.display.init()

        # Manipulators
        self.manip_publisher = self.create_publisher(
            Manip, 'manipulator_control', qos_profile_system_default
        )

        self.manip_buttons: dict[int, ManipButton] = {
            self.profile.manip_left: ManipButton('left'),
            self.profile.manip_right: ManipButton('right'),
        }

        self.get_logger().info('Connecting to mavlink...')
        self.mavlink = mavutil.mavlink_connection(MAVLINK_CONNECTION_STRING, source_system=255)

        self.state_publisher = self.create_publisher(
            VehicleState, 'vehicle_state_event', qos_profile_system_default
        )

        self.heartbeat_subscription = self.create_subscription(
            Heartbeat, 'pi_heartbeat', self.pi_heartbeat_callback, 10
        )

        self.arming_service = self.create_service(
            VehicleArming, 'arming', callback=self.arming_service_callback
        )

        self.invert_controls = False

        # Unix timestamp of the last mavlink heartbeat from the pi
        self.last_pi_heartbeat: float = 0
        self.last_ardusub_heartbeat: float = 0
        self.last_state_subscriber_count: int = 0

        self.vehicle_state = VehicleState(pi_connected=False, ardusub_connected=False, armed=False)

        self.timer = self.create_timer(1 / MAVLINK_POLL_RATE, self.poll_mavlink)

    def controller_callback(self, joy_state: JoystickState) -> None:
        """Handle a joystick update.

        Parameters
        ----------
        joy_state : JoystickState
            The current state of the joystick buttons and axes
        """
        self.process_arming_buttons(joy_state)
        self.send_mavlink_control(joy_state)
        self.manip_callback(joy_state)
        self.process_camera_buttons(joy_state)

    @staticmethod
    def joystick_map(raw: float) -> float:
        """Apply a mapping to a joystick axis before it is used for control.

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

    def send_mavlink_control(self, joy_state: JoystickState) -> None:
        """Send a mavlink manual control message to the vehcile based on the joystick inputs.

        Parameters
        ----------
        joy_state : JoystickState
            The state of the joystick buttons and axes
        """
        axes = joy_state.axes
        buttons = joy_state.buttons

        inv = -1 if self.invert_controls else 1

        self.mavlink.mav.manual_control_send(
            self.mavlink.target_system,
            int(-self.joystick_map(axes[self.profile.forward]) * 1000) * inv,
            int(self.joystick_map(axes[self.profile.lateral]) * 1000) * inv,
            int(
                (
                    self.joystick_map(axes[self.profile.vertical_up] / 2 + 0.5)
                    - self.joystick_map(axes[self.profile.vertical_down] / 2 + 0.5)
                )
                / 2
                * 1000
                + 500
            ),
            int(self.joystick_map(axes[self.profile.yaw]) * 1000),
            0,
            0,
            MANUAL_CONTROL_EXTENSIONS_CODE,
            int(-self.joystick_map(axes[self.profile.pitch]) * PITCH_THROTTLE * 1000) * inv,
            int((buttons[self.profile.roll_left] - buttons[self.profile.roll_right]) * 1000) * inv,
        )

    def manip_callback(self, joy_state: JoystickState) -> None:
        """Process a joystick state and send a ros message to open or close a manipulator if
        required.

        Parameters
        ----------
        joy_state : JoystickState
            The state of the joystick buttons and axes
        """
        buttons = joy_state.buttons
        for button_id, manip_button in self.manip_buttons.items():
            just_pressed = buttons[button_id] == PRESSED
            if manip_button.last_button_state is False and just_pressed:
                new_manip_state = not manip_button.is_active
                manip_button.is_active = new_manip_state
                manip_msg = Manip(manip_id=manip_button.claw, activated=manip_button.is_active)
                self.manip_publisher.publish(manip_msg)

            manip_button.last_button_state = just_pressed

    def set_armed(self, *, arm: bool) -> None:
        """Send a mavlink message to arm or disarm the vehicle.

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
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def arming_service_callback(
        self, request: VehicleArming.Request, response: VehicleArming.Response
    ) -> VehicleArming.Response:
        """Handle a request to arm or disarm the robot.

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
        self.set_armed(arm=request.arm)

        response.message_sent = True
        return response

    def process_arming_buttons(self, joy_state: JoystickState) -> None:
        """Set the arming state using the menu and pairing buttons.

        Parameters
        ----------
        joy_state : JoystickState
            The state of the joystick buttons and axes
        """
        buttons = joy_state.buttons

        if buttons[self.profile.disarm_button] == PRESSED:
            self.get_logger().info('Sending disarm command')
            self.set_armed(arm=False)
        elif buttons[self.profile.arm_button] == PRESSED:
            self.get_logger().info('Sending arm command')
            self.set_armed(arm=True)

    def process_camera_buttons(self, joy_state: JoystickState) -> None:
        """Switch cameras and invert control when camera switch buttons are pressed.

        Parameters
        ----------
        joy_state : JoystickState
            The state of the joystick buttons and axess
        """
        # Camera switching uses the DPAD, currently not remapable with the controller profile system
        # because DPADs are presented as axes not buttons and using any other axis is non-sensible
        if joy_state.buttons[self.profile.cam_front_button]:
            # TODO: Message camera manager and gui to swap cameras
            self.invert_controls = False
        elif joy_state.buttons[self.profile.cam_back_button]:
            # TODO: Message camera manager and gui to swap cameras
            self.invert_controls = True

    def poll_mavlink(self) -> None:
        """Check for incoming mavlink messages from the vehicle and send state updates if
        the vehicle state has changed.
        """
        new_state = self.poll_mavlink_for_new_state()

        # Check pi timeeout
        if self.vehicle_state.pi_connected and time.time() - self.last_pi_heartbeat > PI_TIMEOUT:
            self.vehicle_state.pi_connected = False
            self.state_publisher.publish(self.vehicle_state)
            self.get_logger().warning('Pi disconnected')

        # Check ardusub timeeout
        if (
            self.vehicle_state.ardusub_connected
            and time.time() - self.last_ardusub_heartbeat > ARDUSUB_TIMEOUT
        ):
            new_state.ardusub_connected = False
            self.state_publisher.publish(self.vehicle_state)
            self.get_logger().warning('Ardusub disconnected')

        if self.vehicle_state != new_state:
            self.vehicle_state = new_state
            self.state_publisher.publish(new_state)

    def poll_mavlink_for_new_state(self) -> VehicleState:
        """Read incoming mavlink messages to determine the state of the vehicle.

        Returns
        -------
        VehicleState
            The updated state of the vehicle
        """
        new_state = VehicleState(
            pi_connected=self.vehicle_state.pi_connected,
            ardusub_connected=self.vehicle_state.ardusub_connected,
            armed=self.vehicle_state.armed,
        )

        # Walrus operator <3 <3
        while mavlink_msg := self.mavlink.recv_match():
            if (
                mavlink_msg.get_srcComponent() != VEHICLE_COMPONENT_ID
                or mavlink_msg.get_type() != 'HEARTBEAT'
            ):
                continue

            self.last_ardusub_heartbeat = time.time()

            new_state.ardusub_connected = True
            if not self.vehicle_state.ardusub_connected:
                self.get_logger().info('Ardusub connected')

            if mavlink_msg.system_status == mavutil.mavlink.MAV_STATE_ACTIVE:
                new_state.armed = True
                if not self.vehicle_state.armed:
                    self.get_logger().info('Vehicle armed')
            elif mavlink_msg.system_status == mavutil.mavlink.MAV_STATE_STANDBY:
                new_state.armed = False
                if self.vehicle_state.armed:
                    self.get_logger().info('Vehicle disarmed')
            else:
                self.get_logger().warning(f'Unknown ardusub state: {mavlink_msg.system_status}')
        return new_state

    def pi_heartbeat_callback(self, _: Heartbeat) -> None:
        """Handle a ros heartbeat message from the Pi on the vehicle.

        Parameters
        ----------
        _ : Heartbeat
            The heartbeat message
        """
        self.last_pi_heartbeat = time.time()

        if not self.vehicle_state.pi_connected:
            self.vehicle_state.pi_connected = True
            self.state_publisher.publish(self.vehicle_state)
            self.get_logger().info('Pi connected')

    def poll_subscribers(self) -> None:
        """Check for ros subsribers to our vehicle state topic, and send a state update whenever
        a new one subscribes.
        """
        # Whenever a node subscribes to vehicle state updates, send the current state
        subscriber_count = self.state_publisher.get_subscription_count()
        if subscriber_count > self.last_state_subscriber_count:
            self.state_publisher.publish(self.vehicle_state)

        self.last_state_subscriber_count = subscriber_count

    def handle_pygame_events(self) -> None:
        """Poll pygame for joystick connection and disconnection events."""
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                new_joy = joystick.Joystick(event.device_index)
                self.joysticks[new_joy.get_instance_id()] = new_joy
                if not self.current_joystick_id:
                    self.current_joystick_id = new_joy.get_instance_id()
                self.get_logger().info(f'Joystick {new_joy.get_instance_id()} connected')

            elif event.type == pygame.JOYDEVICEREMOVED:
                del self.joysticks[event.instance_id]
                if self.current_joystick_id == event.instance_id:
                    if self.joysticks:
                        # Arbitrarily choose a new joystick to become the current joystick
                        # In practice this is the first element of list(self.joystick.keys())
                        self.current_joystick_id = next(iter(self.joysticks))
                    else:
                        self.current_joystick_id = None
                self.get_logger().info(f'Joystick {event.instance_id} disconnected')

    def poll_joystick(self) -> None:
        """Read the current state of the joystick and send a mavlink message."""
        self.handle_pygame_events()

        if self.current_joystick_id is not None:
            joy = self.joysticks[self.current_joystick_id]

            axes = [joy.get_axis(ax) for ax in range(joy.get_numaxes())]
            buttons = [joy.get_button(btn) for btn in range(joy.get_numbuttons())]

            # Represent dpad movements as button presses
            for hat_idx in range(joy.get_numhats()):
                for hat_axis in joy.get_hat(hat_idx):
                    if hat_axis > 0:
                        buttons += [False, True]
                    elif hat_axis < 0:
                        buttons += [True, False]
                    else:
                        buttons += [False, False]

            self.controller_callback(JoystickState(axes=axes, buttons=buttons))


def main() -> None:
    """Instantiate a mavlink manual control node, and constantly poll the joystick,
    mavlink, and new subscribers in a loop.
    """
    rclpy.init()
    manual_control = MavlinkManualControlNode()
    executor = MultiThreadedExecutor()

    last_loop_start = time.time()
    last_mav_poll = time.time()
    last_sub_poll = time.time()

    loop_period = 1 / JOYSTICK_POLL_RATE
    mav_poll_period = 1 / MAVLINK_POLL_RATE
    sub_poll_period = 1 / SUBSCRIBER_POLL_RATE

    while rclpy.utilities.ok():
        loop_start = time.time()
        rclpy.spin_once(manual_control, executor=executor, timeout_sec=0)

        if loop_start - last_mav_poll > mav_poll_period:
            manual_control.poll_mavlink()
            last_mav_poll += mav_poll_period

        elif loop_start - last_sub_poll > sub_poll_period:
            manual_control.poll_subscribers()
            last_sub_poll += sub_poll_period

        manual_control.poll_joystick()

        sleep_time = last_loop_start + loop_period - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
            last_loop_start += loop_period
        else:
            manual_control.get_logger().warning(f'Loop overrun: {-sleep_time}')
            last_loop_start = loop_start
