from launch.actions import GroupAction
from launch.launch_description import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    keyboard_control_node = Node(
        package='flight_control',
        executable='keyboard_control_node',
        emulate_tty=True,
        output='screen'
    )

    control_inverter_node = Node(
        package='flight_control',
        executable='control_inverter_node',
        emulate_tty=True,
        output='screen'
    )

    multiplexer_node = Node(
        package='flight_control',
        executable='multiplexer',
        remappings=[('/surface/mavros/rc/override', '/tether/mavros/rc/override')],
        emulate_tty=True,
        output='screen'
    )

    namespace_launch: GroupAction = GroupAction(
        actions=[
            PushRosNamespace("surface"),
            keyboard_control_node,
            control_inverter_node,
            multiplexer_node
        ]
    )

    return LaunchDescription([
        namespace_launch
    ])
