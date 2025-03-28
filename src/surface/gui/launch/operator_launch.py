from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    gui_node = Node(
        package='gui',
        executable='run_operator',
        parameters=[{'theme': LaunchConfiguration('theme', default='dark')}],
        remappings=[
            # ('/surface/mavros/cmd/command', '/tether/mavros/cmd/command'),
            # ('/surface/mavros/param/set', '/tether/mavros/param/set'),
            # ('/surface/mavros/param/pull', '/tether/mavros/param/pull'),
            ('/surface/temperature', '/tether/temperature'),
            # ('/surface/mavros/cmd/arming', '/tether/mavros/cmd/arming'),
            ('/surface/ip_address', '/tether/ip_address'),
            ('/surface/flooding', '/tether/flooding'),
        ],
        emulate_tty=True,
        output='screen',
    )

    timer_node = Node(package='gui', executable='run_timer')

    return LaunchDescription([gui_node, timer_node])
