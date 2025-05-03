"""pi_launch launch file."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace

NAMESPACE = 'pi'


def generate_launch_description() -> LaunchDescription:
    # Manipulator Controller
    manip_path = get_package_share_directory('manipulators')

    manip_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(Path(manip_path) / 'launch' / 'manip_launch.py')])
    )

    # Pi Info
    pi_info_path = get_package_share_directory('pi_info')

    pi_info_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(Path(pi_info_path) / 'launch' / 'pi_info_launch.py')])
    )

    # Flood detection
    flood_sensors_path = get_package_share_directory('flood_detection')

    flood_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(Path(flood_sensors_path) / 'launch' / 'flood_detection_launch.py')]
        )
    )

    namespace_launch = GroupAction(
        actions=[
            PushRosNamespace(NAMESPACE),
            manip_launch,
            flood_detection_launch,
            pi_info_launch,
        ]
    )

    return LaunchDescription(
        [
            namespace_launch,
        ]
    )
