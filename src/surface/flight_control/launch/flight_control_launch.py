from launch.launch_description import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    manual_control_node = Node(
        package='flight_control',
        executable='manual_control_node',
        parameters=[
            {'controller_mode': LaunchConfiguration('controller_mode', default=0)},
            {'controller_profile': LaunchConfiguration('controller_profile', default=0)},
        ],
        remappings=[
            ('/surface/manipulator_control', '/tether/manipulator_control'),
            ('/surface/mavros/cmd/arming', '/tether/mavros/cmd/arming'),
            ('/surface/mavros/manual_control/send', '/tether/mavros/manual_control/send'),
            ('/surface/mavros/cmd/command', '/tether/mavros/cmd/command'),
        ],
        emulate_tty=True,
        output='screen',
    )

    return LaunchDescription(
        [manual_control_node]
    )
