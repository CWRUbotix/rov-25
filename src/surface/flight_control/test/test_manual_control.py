import rclpy
from flight_control.manual_control_node import ManualControlNode
from flight_control.multiplexer import (FORWARD_CHANNEL, LATERAL_CHANNEL,
                                        PITCH_CHANNEL, RANGE_SPEED,
                                        ROLL_CHANNEL, THROTTLE_CHANNEL,
                                        YAW_CHANNEL, ZERO_SPEED,
                                        MultiplexerNode)

from rov_msgs.msg import PixhawkInstruction


def test_manual_control_instantiation() -> None:
    """Unit test for the Manual Control instantiation."""
    rclpy.init()
    ManualControlNode()
    rclpy.shutdown()


def test_joystick_profiles() -> None:
    """Unit test for the joystick_profiles function."""
    instruction = PixhawkInstruction(
        # Nice boundary values
        forward=0,
        vertical=1,
        lateral=-1,

        # Not nice possible values
        pitch=0.34,
        yaw=-0.6,
        roll=0.92
    )

    msg = MultiplexerNode.to_override_rc_in(instruction)

    assert msg.channels[FORWARD_CHANNEL] == ZERO_SPEED
    assert msg.channels[THROTTLE_CHANNEL] == (ZERO_SPEED + RANGE_SPEED)
    assert msg.channels[LATERAL_CHANNEL] == (ZERO_SPEED - RANGE_SPEED)

    # 1539 1378

    assert msg.channels[PITCH_CHANNEL] == ZERO_SPEED + int(RANGE_SPEED * 0.34)
    assert msg.channels[YAW_CHANNEL] == ZERO_SPEED + int(RANGE_SPEED * -0.6)
    assert msg.channels[ROLL_CHANNEL] == ZERO_SPEED + int(RANGE_SPEED * 0.92)
