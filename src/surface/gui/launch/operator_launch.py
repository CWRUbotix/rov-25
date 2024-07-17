from launch.actions import GroupAction
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    """Asynchronously launches operator's gui node."""
    gui_node = Node(
        package='gui',
        executable='run_operator',
        parameters=[{'theme': LaunchConfiguration('theme', default='dark')}],
        remappings=[
            ('/surface/gui/mavros/cmd/command', '/tether/mavros/cmd/command'),
            ('/surface/gui/mavros/param/set', '/tether/mavros/param/set'),
            ('/surface/gui/mavros/param/pull', '/tether/mavros/param/pull'),
            ('/surface/gui/temperature', '/tether/temperature'),
            ('/surface/gui/mavros/cmd/arming', '/tether/mavros/cmd/arming'),
            ('/surface/gui/ip_address', '/tether/ip_address'),
            ('/surface/gui/flooding', '/tether/flooding'),
        ],
        emulate_tty=True,
        output='screen',
    )

    timer_node = Node(package='gui', executable='run_timer')

    return LaunchDescription([gui_node, timer_node])
