from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    gui_path = get_package_share_directory('gui')
    controller_path = get_package_share_directory('ps5_controller')
    # flir_path: str = get_package_share_directory('rov_flir')

    simulation_configuration = LaunchConfiguration('simulation', default=False)

    # Launches Gui
    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(Path(gui_path) / 'launch' / 'pilot_launch.py')]),
    )

    # Launches Controller
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(Path(controller_path) / 'launch' / 'controller_launch.py')]
        ),
    )

    # Launches flir
    # flir_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             flir_path, 'launch', 'flir_launch.py'
    #         )
    #     ]),
    #     condition=UnlessCondition(simulation_configuration)
    # )

    flir_watchdog = Node(
        package='rov_flir',
        executable='flir_watchdog',
        name='flir_watchdog',
        emulate_tty=True,
        output='screen',
        condition=UnlessCondition(simulation_configuration),
    )

    namespace_launch = GroupAction(
        actions=[PushRosNamespace('surface'), gui_launch, controller_launch, flir_watchdog]
    )

    return LaunchDescription([namespace_launch])
