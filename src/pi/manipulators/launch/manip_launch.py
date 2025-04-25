from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    manip_node = Node(
        package='manipulators',
        executable='manipulators',
        parameters=[
            {'left': 3},
            {'right': 2},
            # {"light": 2},
        ],
        remappings=[('/pi/manipulator_control', '/tether/manipulator_control')],
        emulate_tty=True,
        output='screen',
    )

    return LaunchDescription([manip_node])
