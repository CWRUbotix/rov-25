from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # launches node to capture joystick data
    controller_node = Node(package='joy', executable='joy_node', emulate_tty=True, output='screen')

    return LaunchDescription(
        [
            controller_node,
        ]
    )
