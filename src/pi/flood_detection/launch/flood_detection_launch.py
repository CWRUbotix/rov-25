from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    flood_detection = Node(
        package='flood_detection',
        executable='flood_detector',
        emulate_tty=True,
        output='screen',
        remappings=[('/pi/flooding', '/tether/flooding')],
    )

    return LaunchDescription([flood_detection])
