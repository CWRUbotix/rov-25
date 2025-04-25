from launch.actions import GroupAction
from launch.launch_description import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    dual_cam = Node(
        package='luxonis_cam',
        executable='luxonis_cam_driver',
        name='luxonis_cam',
        exec_name='luxonis_cam',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'cam_to_stream': LaunchConfiguration('cam_to_stream', default='left')}
        ]
    )

    namespace_launch = GroupAction(
        actions=[
            PushRosNamespace('surface'),
            dual_cam
        ]
    )

    return LaunchDescription([namespace_launch])
