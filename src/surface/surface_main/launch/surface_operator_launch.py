from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    gui_path = get_package_share_directory('gui')
    flight_control_path = get_package_share_directory('flight_control')
    transceiver_path = get_package_share_directory('transceiver')
    photosphere_path = get_package_share_directory('photosphere')

    # Launches Gui
    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(Path(gui_path) / 'launch' / 'operator_launch.py')]),
    )

    # Launches flight_control (auto docking, manual control, etc.)
    flight_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(Path(flight_control_path) / 'launch' / 'flight_control_launch.py')]
        ),
    )

    # Launches Transceiver
    transceiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(Path(transceiver_path) / 'launch' / 'serial_reader_launch.py')]
        ),
    )

    # Launches Photosphere
    photosphere_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(Path(photosphere_path) / 'launch' / 'photosphere_launch.py')]
        ),
    )

    namespace_launch = GroupAction(
        actions=[
            PushRosNamespace('surface'),
            gui_launch,
            flight_control_launch,
            transceiver_launch,
            photosphere_launch,
        ]
    )

    return LaunchDescription([namespace_launch])
