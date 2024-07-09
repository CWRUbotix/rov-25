import pytest
import rclpy
from flight_control.keyboard_control_node import KeyboardListenerNode


@pytest.fixture
def test_keyboard_listener_instantiation() -> None:
    """Unit test for KeyboardListenerNode instantiation."""
    rclpy.init()
    KeyboardListenerNode()
    rclpy.shutdown()
