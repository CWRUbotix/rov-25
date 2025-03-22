import rclpy
from mavros_msgs.msg import ManualControl

from flight_control.manual_control_node import ManualControlNode
from flight_control.manual_control_utils import (
    JOYSTICK_EXPONENT,
    ZERO_SPEED,
    Z_ZERO_SPEED,
    Z_RANGE_SPEED,
    RANGE_SPEED,
    apply_function,
    joystick_map,
)

def test_manual_control_instantiation() -> None:
    """Unit test for the Manual Control instantiation."""
    rclpy.init()
    ManualControlNode()
    rclpy.shutdown()

def test_joystick_map() -> None:
    """Unit test for the joystick_map function."""
    instruction = ManualControl(
        # Nice boundary values
        x=0,
        z=1,
        y=-1,
        # Not nice possible values
        s=0.34,
        r=-0.6,
        t=0.92,
    )

    msg = apply_function(instruction, joystick_map)
    msg.z = Z_RANGE_SPEED * instruction.z + Z_ZERO_SPEED


    assert msg.x == ZERO_SPEED
    assert msg.z == (Z_ZERO_SPEED + Z_RANGE_SPEED)
    assert msg.y == (ZERO_SPEED - RANGE_SPEED)
 
    # 1539 1378

    assert msg.s == ZERO_SPEED + RANGE_SPEED * (0.34 ** JOYSTICK_EXPONENT)
    assert msg.r == ZERO_SPEED + RANGE_SPEED * (-0.6 ** JOYSTICK_EXPONENT)
    assert msg.t == ZERO_SPEED + RANGE_SPEED * (0.92 ** JOYSTICK_EXPONENT)