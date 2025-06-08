import paramiko
import os

import rclpy
import rclpy.utilities
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory


HOST = "192.168.2.5"
USER = "rov"
PASSWORD = "rov12345"

TAKE_PICS_CMD = "bash /home/rov/rov-25/src/photosphere/take_images.sh"
LOCAL_PATH = "src/surface/photosphere/images"  # relative the rov-25 repo

REMOTE_PATH = "/home/rov/rov-25/src/photosphere/images"

class PhotosphereDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('photosphere_driver_node')

        self.arming_service = self.create_service(
            Trigger, 'take_photos', callback=self.take_photos_callback
        )

        self.local_images_path = os.path.join(get_package_share_directory("photosphere").split("rov-25")[0], "rov-25", LOCAL_PATH)
        print(self.local_images_path)

    def take_photos_callback(
            self, request: Trigger.Request, response: Trigger.Response
        ) -> Trigger.Response:
        """Connect to the photosphere over ssh and take an image from each camera

        Parameters
        ----------
        request : Trigger.Request
            The ROS service request; Trigger has no request fields
        response : Trigger.Response
            The ROS service response (success and message)

        Returns
        -------
        Trigger.Response
            The filled ROS service response
        """

        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh_client.connect(HOST, username=USER, password=PASSWORD)
        self.get_logger().info("Connected to Calamari")

        _, ssh_stdout, _ = ssh_client.exec_command(TAKE_PICS_CMD)
        ssh_stdout.channel.set_combine_stderr(True)

        for line in ssh_stdout:
            self.get_logger().info(line.strip())

        ftp_client = ssh_client.open_sftp()
        ftp_client.get(os.path.join(REMOTE_PATH, "cam0.jpg"), os.path.join(self.local_images_path, "cam0.jpg"))
        ftp_client.get(os.path.join(REMOTE_PATH, "cam1.jpg"), os.path.join(self.local_images_path, "cam1.jpg"))

        self.get_logger().info("Images downloaded from Calamari")

        response = Trigger.Response()
        response.success = True

        return response


def main() -> None:
    """Run the photosphere driver node."""
    rclpy.init()

    node = PhotosphereDriverNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()