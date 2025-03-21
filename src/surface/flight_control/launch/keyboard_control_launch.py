from launch.launch_description import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    keyboard_control_node = Node(
        package='flight_control',
        executable='keyboard_control_node',
        parameters=[
            {'controller_mode': LaunchConfiguration('controller_mode', default=0)},
            {'controller_profile': LaunchConfiguration('controller_profile', default=0)},
        ],
        remappings=[('/surface/mavros/manual_control/send', '/tether/mavros/manual_control/send')],
        emulate_tty=True,
        output='screen',
    )

    return LaunchDescription([keyboard_control_node])
