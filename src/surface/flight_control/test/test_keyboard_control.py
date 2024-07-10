import rclpy
from flight_control.keyboard_control_node import KeyboardListenerNode


def test_keyboard_listener_instantiation() -> None:
    """Unit test for KeyboardListenerNode instantiation."""
    rclpy.init()
    KeyboardListenerNode()
    rclpy.shutdown()
