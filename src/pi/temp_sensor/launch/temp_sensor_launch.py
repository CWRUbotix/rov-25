from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generate LaunchDescription for temp_sensor.

    Returns
    -------
    LaunchDescription
        Launches temperature sensor node

    """
    temp_sensor = Node(
        package='temp_sensor',
        executable='temp_sensor',
        emulate_tty=True,
        output='screen',
        remappings=[('/pi/temperature', '/tether/temperature')]
    )

    return LaunchDescription([
        temp_sensor
    ])
