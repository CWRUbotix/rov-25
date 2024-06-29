import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import Parameter
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import GroupAction


def generate_launch_description() -> LaunchDescription:
    front_arg = LaunchConfiguration('launch_front', default=True)
    bottom_arg = LaunchConfiguration('launch_bottom', default=True)
    ns_arg = LaunchConfiguration('ns', default='')

    parameter_file = os.path.join(get_package_share_directory('rov_flir'), 'config',
                                  'blackfly_s.yaml')

    parameters = {
        'debug': False,
        'acquisition_timeout': 0.5,
        'compute_brightness': False,
        'adjust_timestamp': True,
        'dump_node_map': False,
        # set parameters defined in blackfly_s.yaml
        'gain_auto': 'Continuous',
        'pixel_format': 'BayerRG8',
        'exposure_auto': 'Continuous',
        # These are useful for GigE cameras
        'device_link_throughput_limit': 125000000,
        'gev_scps_packet_size': 9000,
        # ---- to reduce the sensor width and shift the crop
        'image_width': 720,
        'image_height': 540,
        # 'offset_x': 16,
        # 'offset_y': 0,
        'binning_x': 2,
        'binning_y': 2,
        'connect_while_subscribed': True,
        'frame_rate_auto': 'Off',
        'frame_rate': 60.0,
        'frame_rate_enable': True,
        'buffer_queue_size': 1,
        'trigger_mode': 'Off',
        'chunk_mode_active': True,
        'chunk_selector_frame_id': 'FrameID',
        'chunk_enable_frame_id': True,
        'chunk_selector_exposure_time': 'ExposureTime',
        'chunk_enable_exposure_time': True,
        'chunk_selector_gain': 'Gain',
        'chunk_enable_gain': True,
        'chunk_selector_timestamp': 'Timestamp',
        'chunk_enable_timestamp': True
    }

    # launches node to run front flir camera
    front_cam = Node(
        package='spinnaker_camera_driver',
        executable='camera_driver_node',
        name='front_cam',
        exec_name='front_cam',
        emulate_tty=True,
        output='screen',
        parameters=[Parameter('serial_number', '23473566'),
                    Parameter('parameter_file', parameter_file),
                    parameters],
        condition=IfCondition(front_arg)
    )

    # launches node to run bottom flir camera
    bottom_cam = Node(
        package='spinnaker_camera_driver',
        executable='camera_driver_node',
        name='bottom_cam',
        exec_name='bottom_cam',
        emulate_tty=True,
        output='screen',
        parameters=[Parameter('serial_number', '23473577'),
                    Parameter('parameter_file', parameter_file),
                    parameters],
        condition=IfCondition(bottom_arg)
    )

    namespace_launch = GroupAction(
        actions=[
            PushRosNamespace(ns_arg),
            front_cam,
            bottom_cam
        ]
    )

    return LaunchDescription([namespace_launch])
