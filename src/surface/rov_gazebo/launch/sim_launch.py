import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

NAMESPACE = "simulation"


def generate_launch_description() -> LaunchDescription:
    rov_gazebo_path: str = get_package_share_directory("rov_gazebo")
    surface_main_path: str = get_package_share_directory("surface_main")

    world_file = "rov24_coral.sdf"
    world_path: str = os.path.join(rov_gazebo_path, "worlds", world_file)

    params_file = "sub.parm"
    params_path: str = os.path.join(rov_gazebo_path, "config", params_file)

    # TODO gz_sim launch might be nice
    start_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '3', '-r', world_path],
        output='screen'
    )

    start_ardusub = ExecuteProcess(
        cmd=["ardusub", '-S', '-w', '-M', 'JSON', '--defaults', params_path, '-I0', ],
        output='screen'
    )

    # Translate messages MAV <-> ROS
    mav_ros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        namespace='mavros',
        parameters=[
            {"system_id": 255},
            {"fcu_url": "tcp://localhost"},
            {"gcs_url": "udp://@localhost:14550"},
            {"plugin_allowlist": ["rc_io", "sys_status", "command"]}
        ],
        remappings=[
            (f'/{NAMESPACE}/mavros/state', '/tether/mavros/state'),
            (f'/{NAMESPACE}/mavros/rc/override', '/tether/mavros/rc/override'),
            (f'/{NAMESPACE}/mavros/cmd/arming', '/tether/mavros/cmd/arming'),
            (f'/{NAMESPACE}/mavros/cmd/command', '/tether/mavros/cmd/command'),
        ],
        emulate_tty=True
    )

    cam_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge",
        arguments=["front_cam", "bottom_cam"],
        output="screen",
        remappings=[
            (f"/{NAMESPACE}/front_cam", "/surface/front_cam/image_raw"),
            (f"/{NAMESPACE}/bottom_cam", "/surface/bottom_cam/image_raw"),
        ],
    )

    keyboard_control_node = Node(
        package="flight_control",
        executable="keyboard_control_node",
        namespace="surface",
        output="screen",
        name="keyboard_control_node",
        emulate_tty=True
    )

    # Launches the pi heartbeat node
    heartbeat_node = Node(
        package="pi_info",
        executable="heartbeat_node",
        remappings=[(f"/{NAMESPACE}/pi_heartbeat", "/tether/pi_heartbeat")],
        emulate_tty=True,
        output="screen"
    )

    # Launches the ip address node
    ip_node = Node(
        package="pi_info",
        executable="ip_publisher",
        remappings=[(f"/{NAMESPACE}/ip_address", "/tether/ip_address")],
        emulate_tty=True,
        output="screen"
    )

    # Launches Surface Nodes
    surface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(surface_main_path, "launch", "surface_all_nodes_launch.py")]
        ),
        launch_arguments=[('simulation', 'true'),
                          ('gui', 'debug')]
    )

    namespace_launch = GroupAction(
        actions=[
            PushRosNamespace(NAMESPACE),
            mav_ros_node,
            heartbeat_node,
            cam_bridge_node,
            ip_node
        ]
    )

    return LaunchDescription(
        [
            keyboard_control_node,
            start_gazebo,
            start_ardusub,
            namespace_launch,
            surface_launch,
        ]
    )
