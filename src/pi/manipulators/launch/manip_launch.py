from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    manip_node = Node(
        package="manipulators",
        executable="manipulators",
        parameters=[
            {"left": 2},
            {"right": 3},
            # {"light": 2},
        ],
        remappings=[("/pi/manipulator_control", "/tether/manipulator_control")],
        emulate_tty=True,
        output="screen"
    )

    lgpio_manip_node = Node(
        package="manipulators",
        executable="lgpio_manipulator",
        remappings=[("/pi/manipulator_control", "/tether/manipulator_control")],
        emulate_tty=True,
        output="screen"
    )

    valve_manip_node = Node(
        package="manipulators",
        executable="valve_manipulator",
        remappings=[("/pi/valve_manipulator", "/tether/valve_manipulator")],
        emulate_tty=True,
        output="screen"
    )

    return LaunchDescription([
        manip_node,
        lgpio_manip_node,
        valve_manip_node
    ])
