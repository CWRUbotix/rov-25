from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    surface_path = get_package_share_directory('surface_main')

    all_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(Path(surface_path) / 'launch' / 'surface_all_nodes_launch.py')]
        ),
        launch_arguments=[('gui', 'debug')],
    )

    return LaunchDescription(
        [
            all_launch,
        ]
    )
