from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    manip_node = Node(
        package="manipulators",
        executable="manipulators",
        parameters=[
            {"left": 0},
            {"right": 1},
            # {"light": 2},
        ],
        remappings=[("/pi/manipulator_control", "/tether/manipulator_control")]
    )

    return LaunchDescription([
        manip_node
    ])