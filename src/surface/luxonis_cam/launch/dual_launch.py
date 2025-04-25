from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    luxonis_cam = get_package_share_directory('luxonis_cam')

    left_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(Path(luxonis_cam) / 'launch' / 'mono_launch.py')]
        ),
        launch_arguments=[('cam_to_stream', 'left')],
    )

    right_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(Path(luxonis_cam) / 'launch' / 'mono_launch.py')]
        ),
        launch_arguments=[('cam_to_stream', 'right')],
    )

    return LaunchDescription(
        [
            left_cam,
            right_cam
        ]
    )
