from typing import TYPE_CHECKING

import rclpy.utilities
from mavros_msgs.msg import ManualControl
from pynput.keyboard import Key, KeyCode, Listener
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from flight_control.manual_control_utils import (
    Z_RANGE_SPEED,
    Z_ZERO_SPEED,
    apply_function,
    joystick_map,
)

if TYPE_CHECKING:
    from rclpy.publisher import Publisher

# key bindings
FORWARD = 'w'
BACKWARD = 's'
LEFT = 'a'
RIGHT = 'd'
UP = '2'
DOWN = 'x'

ROLL_LEFT = 'j'
ROLL_RIGHT = 'l'
PITCH_UP = 'i'
PITCH_DOWN = 'k'
YAW_LEFT = 'h'
YAW_RIGHT = ';'

HELP = 'p'

HELP_MSG = """
Use keyboard to control ROV

Key Bindings:
[2]
[w]            [i]
[a][s][d]   [h][j][k][l][;]
[x]

[w] = Forward
[s] = Backward
[a] = Left
[d] = Right
[2] = Up
[x] = Down

[j] = Roll Left
[l] = Roll Right
[i] = Pitch Up
[k] = Pitch Down
[h] = Yaw Left
[;] = Yaw Right

[p] = Show this help"""


class KeyboardListenerNode(Node):
    def __init__(self) -> None:
        super().__init__('keyboard_listener_node', parameter_overrides=[])

        # Manual Control
        self.mc_pub: Publisher = self.create_publisher(
            ManualControl, 'mavros/manual_control/send', QoSPresetProfiles.DEFAULT.value
        )

        self.get_logger().info(HELP_MSG)
        self.status = {
            FORWARD: False,
            BACKWARD: False,
            LEFT: False,
            RIGHT: False,
            UP: False,
            DOWN: False,
            ROLL_LEFT: False,
            ROLL_RIGHT: False,
            PITCH_UP: False,
            PITCH_DOWN: False,
            YAW_LEFT: False,
            YAW_RIGHT: False,
        }

    def on_press(self, key: Key | KeyCode | None) -> None:
        if isinstance(key, KeyCode):
            if key.char is None:
                return
            key_name = key.char
        elif isinstance(key, Key):
            key_name = key.name
        else:
            return

        if key_name == HELP:
            self.get_logger().info(HELP_MSG)
        else:
            self.status[key_name] = True

        self.pub_rov_control()

    def on_release(self, key: Key | KeyCode | None) -> None:
        if isinstance(key, KeyCode):
            if key.char is None:
                return
            key_name = key.char
        elif isinstance(key, Key):
            key_name = key.name
        else:
            return

        if key_name == HELP:
            pass
        else:
            self.status[key_name] = False

        self.pub_rov_control()

    def pub_rov_control(self) -> None:
        instruction = ManualControl(
            s=float(self.status[PITCH_UP] - self.status[PITCH_DOWN]),
            t=float(self.status[ROLL_LEFT] - self.status[ROLL_RIGHT]),
            z=float(self.status[UP] - self.status[DOWN]),
            x=float(self.status[FORWARD] - self.status[BACKWARD]),
            y=float(self.status[LEFT] - self.status[RIGHT]),
            r=float(self.status[YAW_LEFT] - self.status[YAW_RIGHT]),
        )

        # Convert to manual_control values
        msg = apply_function(instruction, joystick_map)
        msg.z = Z_RANGE_SPEED * instruction.z + Z_ZERO_SPEED

        self.mc_pub.publish(msg)

    def spin(self) -> None:
        with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            while rclpy.utilities.ok() and listener.running:
                rclpy.spin_once(self, timeout_sec=0.1)


def main() -> None:
    rclpy.init()
    KeyboardListenerNode().spin()


if __name__ == '__main__':
    main()
