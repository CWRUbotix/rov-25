from launch.launch_description import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Asynchronously launches pilot's gui node."""
    pilot_node = Node(
        package='gui',
        executable='run_pilot',
        parameters=[
            {'theme': LaunchConfiguration('theme', default='dark')},
            {'simulation': LaunchConfiguration('simulation', default='false')},
            {'gui': LaunchConfiguration('gui', default='pilot')},
        ],
        remappings=[
            ('/surface/gui/mavros/cmd/arming', '/tether/mavros/cmd/arming'),
            ('/surface/gui/depth_cam/image_raw', '/tether/depth_cam/image_raw'),
        ],
        emulate_tty=True,
        output='screen',
    )

    return LaunchDescription([pilot_node])
