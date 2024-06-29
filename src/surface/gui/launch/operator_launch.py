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
        remappings=[("/surface/gui/bottom_cam/image_raw", "/surface/bottom_cam/image_raw"),
                    ("/surface/gui/auto_control_toggle", "/surface/auto_control_toggle"),
                    ("/surface/gui/mavros/cmd/command", "/tether/mavros/cmd/command"),
                    ("/surface/gui/mavros/param/set", "/tether/mavros/param/set"),
                    ("/surface/gui/mavros/param/pull", "/tether/mavros/param/pull"),
                    ("/surface/gui/temperature", "/tether/temperature"),
                    ("/surface/gui/vehicle_state_event", "/surface/vehicle_state_event"),
                    ("/surface/gui/mavros/cmd/arming", "/tether/mavros/cmd/arming"),
                    ("/surface/gui/ip_address", "/tether/ip_address"),
                    ("/surface/gui/flooding", "/tether/flooding")],
        emulate_tty=True,
        output='screen'
    )

    timer_node: Node = Node(
        package='gui',
        executable='run_timer'
    )

    namespace_launch = GroupAction(
        actions=[
            PushRosNamespace('gui'),
            gui_node,
            timer_node
        ]
    )

    return LaunchDescription([namespace_launch])
