from launch.actions import GroupAction
from launch.launch_description import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    # launches photosphere
    photosphere_node = Node(
        package='photosphere',
        executable='run_photosphere',
        emulate_tty=True,
        output='screen',
    )

    driver_node = Node(
        package='photosphere',
        executable='photosphere_driver',
        emulate_tty=True,
        output='screen',
    )

    namespace_launch = GroupAction(
        actions=[
            PushRosNamespace('photosphere'),
            photosphere_node,
            driver_node,
        ]
    )

    return LaunchDescription([namespace_launch])
