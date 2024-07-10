from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    heartbeat_node = Node(
        package='vehicle_manager',
        executable='connection_manager_node',
        remappings=[
            ('/surface/mavros/state', '/tether/mavros/state'),
            ('/surface/pi_heartbeat', '/tether/pi_heartbeat'),
        ],
        emulate_tty=True,
        output='screen',
    )

    return LaunchDescription([heartbeat_node])
