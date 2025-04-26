from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # launches photosphere
    reader_node = Node(
        package='photosphere',
        executable='serial',
        emulate_tty=True,
        output='screen',
    )

    return LaunchDescription([reader_node])
