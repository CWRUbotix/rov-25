from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Launches the heartbeat_node
    heartbeat_node = Node(
        package='pi_info',
        executable='heartbeat_node',
        remappings=[('/pi/pi_heartbeat', '/tether/pi_heartbeat')],
        emulate_tty=True,
        output='screen',
    )

    # Launches ip_publisher node.
    ip_publisher_node = Node(
        package='pi_info',
        executable='ip_publisher',
        emulate_tty=True,
        output='screen',
        remappings=[('/pi/ip_address', '/tether/ip_address')],
    )

    return LaunchDescription([heartbeat_node, ip_publisher_node])
