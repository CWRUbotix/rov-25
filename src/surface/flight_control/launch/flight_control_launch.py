from launch.launch_description import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    mavlink_node = Node(
        package='flight_control',
        executable='mavlink_control_node',
        parameters=[
            {'controller_mode': LaunchConfiguration('controller_mode', default=0)},
            {'controller_profile': LaunchConfiguration('controller_profile', default=0)},
        ],
        remappings=[
            ('/surface/manipulator_control', '/tether/manipulator_control'),
            ('/surface/pi_heartbeat', '/tether/pi_heartbeat'),
        ],
        emulate_tty=True,
        output='screen',
    )

    # auto_docking_node = Node(
    #     package='flight_control',
    #     executable='auto_docking_node',
    #     remappings=[('/surface/manipulator_control', '/tether/manipulator_control')],
    #     emulate_tty=True,
    #     output='screen',
    # )

    return LaunchDescription([mavlink_node])
