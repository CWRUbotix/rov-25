from launch.actions import GroupAction
from launch.launch_description import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    """Asynchronously launches pilot's gui node."""
    pilot_node = Node(
        package='gui',
        executable='run_pilot',
        parameters=[{'theme': LaunchConfiguration('theme', default='dark')},
                    {'simulation': LaunchConfiguration('simulation', default='false')},
                    {'gui': LaunchConfiguration('gui', default='pilot')}],
        remappings=[("/surface/gui/mavros/cmd/arming", "/tether/mavros/cmd/arming"),
                    ("/surface/gui/camera_switch", "/surface/camera_switch"),
                    ("/surface/gui/bottom_cam/image_raw", "/surface/bottom_cam/image_raw"),
                    ("/surface/gui/front_cam/image_raw", "/surface/front_cam/image_raw"),
                    ("/surface/gui/depth_cam/image_raw", "/tether/depth_cam/image_raw"),
                    ("/surface/gui/vehicle_state_event", "/surface/vehicle_state_event")],
        emulate_tty=True,
        output='screen'
    )

    namespace_launch = GroupAction(
        actions=[
            PushRosNamespace('gui'),
            pilot_node
        ]
    )

    return LaunchDescription([namespace_launch])
