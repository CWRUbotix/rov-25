from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    surface_path = get_package_share_directory('surface_main')
    gui_path = get_package_share_directory('gui')

    all_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(Path(surface_path) / 'launch' / 'surface_all_nodes_launch.py')]
        ),
    )

    livestream_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(Path(gui_path) / 'launch' / 'pilot_launch.py')]),
        launch_arguments=[('gui', 'livestream')],
    )

    namespace_launch = GroupAction(actions=[PushRosNamespace('surface'), livestream_launch])

    return LaunchDescription(
        [
            all_launch,
            namespace_launch,
        ]
    )
