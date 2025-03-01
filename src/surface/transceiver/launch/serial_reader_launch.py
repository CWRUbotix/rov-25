from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # launches transceiver
    reader_node = Node(
        package='transceiver',
        executable='serial',
        emulate_tty=True,
        output='screen',
    )

    return LaunchDescription([reader_node])
