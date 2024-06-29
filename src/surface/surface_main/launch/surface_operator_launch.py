import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace


def generate_launch_description() -> LaunchDescription:

    gui_path: str = get_package_share_directory('gui')
    flight_control_path: str = get_package_share_directory('flight_control')
    vehicle_manager_path: str = get_package_share_directory('vehicle_manager')
    transceiver_path: str = get_package_share_directory('transceiver')

    # Launches Gui
    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                gui_path, 'launch', 'operator_launch.py'
            )
        ]),
    )

    # Launches flight_control (auto docking, manual control, etc.)
    flight_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                flight_control_path, 'launch', 'flight_control_launch.py'
            )
        ]),
    )

    # Launches Vehicle Manager
    vehicle_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                vehicle_manager_path, 'launch', 'vehicle_manager_launch.py'
            )
        ]),
    )

    # Launches Transceiver
    transceiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                transceiver_path, 'launch', 'serial_reader_launch.py'
            )
        ]),
    )

    namespace_launch = GroupAction(
        actions=[
            PushRosNamespace("surface"),
            gui_launch,
            flight_control_launch,
            vehicle_manager_launch,
            transceiver_launch
        ]
    )

    return LaunchDescription([
        namespace_launch
    ])
