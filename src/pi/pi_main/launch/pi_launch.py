"""pi_launch launch file."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace

NAMESPACE = 'pi'


def generate_launch_description() -> LaunchDescription:
    """
    Generate LaunchDescription for pi_main.

    Returns
    -------
    LaunchDescription
        Launches camera_streamer package and pixhawk_communication package.

    """
    # Manipulator Controller
    manip_path = get_package_share_directory('manipulators')

    manip_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(Path(manip_path) / 'launch' / 'manip_launch.py')])
    )

    # Commented out because no usb cams are planned
    # Camera Streamer
    # cam_path = get_package_share_directory('camera_streamer')

    # cam_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             cam_path, 'launch', 'camera_launch.py'
    #         )
    #     ])
    # )

    # Pixhawk Communication
    pixhawk_path = get_package_share_directory('pixhawk_communication')

    pixhawk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                str(Path(pixhawk_path) / 'launch' / 'mavros_launch.py'),
            ]
        )
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

    # Temperature sensor
    temp_sensor_path = get_package_share_directory('temp_sensor')

    temp_sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(Path(temp_sensor_path) / 'launch' / 'temp_sensor_launch.py')]
        )
    )

    namespace_launch = GroupAction(
        actions=[
            PushRosNamespace(NAMESPACE),
            manip_launch,
            pixhawk_launch,
            # cam_launch,
            flood_detection_launch,
            temp_sensor_launch,
            pi_info_launch,
        ]
    )

    return LaunchDescription(
        [
            namespace_launch,
        ]
    )
