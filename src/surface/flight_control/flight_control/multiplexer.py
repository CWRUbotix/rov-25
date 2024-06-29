import rclpy
from rclpy.executors import MultiThreadedExecutor

from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from mavros_msgs.msg import OverrideRCIn
from typing import Callable

from rov_msgs.msg import PixhawkInstruction
from rov_msgs.srv import AutonomousFlight

# Brown out protection
SPEED_THROTTLE = 0.85

# Joystick curve
JOYSTICK_EXPONENT = 3

# Range of values Pixhawk takes
# In microseconds
ZERO_SPEED = 1500
MAX_RANGE_SPEED = 400
RANGE_SPEED = MAX_RANGE_SPEED * SPEED_THROTTLE

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
        super().__init__('multiplexer',
                         parameter_overrides=[])

        self.state = AutonomousFlight.Request.STOP

        self.autonomous_toggle = self.create_service(
            AutonomousFlight,
            'auto_control_toggle',
            self.state_control
        )

        self.control_subscription = self.create_subscription(
            PixhawkInstruction,
            'pixhawk_control',
            self.control_callback,
            QoSPresetProfiles.DEFAULT.value
        )

        self.rc_pub = self.create_publisher(
            OverrideRCIn,
            'mavros/rc/override',
            QoSPresetProfiles.DEFAULT.value
        )

    @staticmethod
    def apply(msg: PixhawkInstruction, function_to_apply: Callable[[float], float]) -> None:
        """Apply a function to each dimension of this PixhawkInstruction."""
        msg.forward = function_to_apply(msg.forward)
        msg.vertical = function_to_apply(msg.vertical)
        msg.lateral = function_to_apply(msg.lateral)
        msg.pitch = function_to_apply(msg.pitch)
        msg.yaw = function_to_apply(msg.yaw)
        msg.roll = function_to_apply(msg.roll)

    @staticmethod
    def to_override_rc_in(msg: PixhawkInstruction) -> OverrideRCIn:
        """Convert this PixhawkInstruction to an rc_msg with the appropriate channels array."""
        rc_msg = OverrideRCIn()

        # Maps to PWM
        MultiplexerNode.apply(msg, lambda value: int(RANGE_SPEED * value) + ZERO_SPEED)

        rc_msg.channels[FORWARD_CHANNEL] = msg.forward
        rc_msg.channels[THROTTLE_CHANNEL] = msg.vertical
        rc_msg.channels[LATERAL_CHANNEL] = msg.lateral
        rc_msg.channels[PITCH_CHANNEL] = msg.pitch
        rc_msg.channels[YAW_CHANNEL] = msg.yaw
        rc_msg.channels[ROLL_CHANNEL] = msg.roll

        return rc_msg

    def state_control(self, req: AutonomousFlight.Request,
                      res: AutonomousFlight.Response) -> AutonomousFlight.Response:
        self.state = req.state
        res.current_state = req.state
        return res

    def control_callback(self, msg: PixhawkInstruction) -> None:

        if msg.author == PixhawkInstruction.MANUAL_CONTROL and \
                self.state == AutonomousFlight.Request.STOP:
            # Smooth out adjustments
            # TODO look into maybe doing inheritance on a PixhawkInstruction
            MultiplexerNode.apply(msg, joystick_map)
        elif msg.author == PixhawkInstruction.KEYBOARD_CONTROL and \
                self.state == AutonomousFlight.Request.STOP:
            pass
        elif msg.author == PixhawkInstruction.AUTONOMOUS_CONTROL and \
                self.state == AutonomousFlight.Request.START:
            pass
        else:
            return

        self.rc_pub.publish(msg=self.to_override_rc_in(msg))


def main() -> None:
    rclpy.init()
    control_invert = MultiplexerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(control_invert, executor=executor)
