from launch.launch_description import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions.launch_configuration import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    manual_control_node = Node(
        package='flight_control',
        executable='manual_control_node',
        parameters=[
            {"controller_mode": LaunchConfiguration('controller_mode', default=0)}
        ],
        remappings=[('/surface/manipulator_control', '/tether/manipulator_control'),
                    ('/surface/valve_manipulator', '/tether/valve_manipulator'),
                    ('/surface/mavros/cmd/arming', '/tether/mavros/cmd/arming')],
        emulate_tty=True,
        output='screen'
    )

    auto_docking_node = Node(
        package='flight_control',
        executable='auto_docking_node',
        remappings=[('/surface/manipulator_control', '/tether/manipulator_control')],
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
        executable='multiplexer_node',
        remappings=[('/surface/mavros/rc/override', '/tether/mavros/rc/override')],
        emulate_tty=True,
        output='screen'
    )

    return LaunchDescription([
        manual_control_node,
        auto_docking_node,
        control_inverter_node,
        multiplexer_node
    ])
