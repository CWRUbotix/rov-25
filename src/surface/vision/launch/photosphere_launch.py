from launch.launch_description import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    photosphere_node = Node(
        package='photosphere',
        executable='photosphere_node',
        parameters=[],
        remappings=[
            # ('/surface/manipulator_control', '/tether/manipulator_control'),
        ],
        emulate_tty=True,
        output='screen',
    )

    return LaunchDescription(
        [photosphere_node]
    )
