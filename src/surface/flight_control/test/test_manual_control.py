import rclpy
from flight_control.manual_control_node import ManualControlNode
from flight_control.multiplexer import (
    RANGE_SPEED,
    ZERO_SPEED,
    Z_ZERO_SPEED,
    Z_RANGE_SPEED,
    MultiplexerNode,
)

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
        roll=0.92,
    )

    msg = MultiplexerNode.to_manual_control(instruction)

    assert msg.x == ZERO_SPEED
    assert msg.z == (Z_ZERO_SPEED + Z_RANGE_SPEED)
    assert msg.y == (ZERO_SPEED - RANGE_SPEED)

    # 1539 1378

    assert msg.s == ZERO_SPEED + int(RANGE_SPEED * 0.34)
    assert msg.r == ZERO_SPEED + int(RANGE_SPEED * -0.6)
    assert msg.t == ZERO_SPEED + int(RANGE_SPEED * 0.92)
