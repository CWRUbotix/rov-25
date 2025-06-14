from pathlib import Path

import cv2
import paramiko
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

HOST = '192.168.2.5'
USER = 'rov'
PASSWORD = 'rov12345'  # noqa: S105
CONNECT_TIMEOUT = 8  # Seconds to log in

TAKE_PICS_CMD = 'bash /home/rov/rov-25/src/photosphere/take_images.sh'
LOCAL_PATH = 'src/surface/photosphere/images'  # relative the rov-25 repo

REMOTE_PATH = '/home/rov/rov-25/src/photosphere/images'


class PhotosphereDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('photosphere_driver_node')

        self.cv_bridge = CvBridge()

        self.arming_service = self.create_service(
            Trigger, 'take_photos', callback=self.take_photos_callback
        )

        self.image_publisher_1 = self.create_publisher(Image, 'image_1', qos_profile_system_default)

        self.image_publisher_2 = self.create_publisher(Image, 'image_2', qos_profile_system_default)

        self.local_images_path = (
            Path(get_package_share_directory('photosphere').split('rov-25')[0])
            / 'rov-25'
            / LOCAL_PATH
        )

        self.get_logger().info('Ready to download photosphere images')

    def take_photos_callback(
        self, _: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Connect to the photosphere over ssh and take an image from each camera.

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
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())  # noqa: S507

        try:
            ssh_client.connect(HOST, username=USER, password=PASSWORD, timeout=CONNECT_TIMEOUT)
        except (TimeoutError, paramiko.ssh_exception.NoValidConnectionsError):
            self.get_logger().error('Failed to connect to photosphere sensor')
            response = Trigger.Response()
            response.success = False
            return response

        self.get_logger().info('Connected to Calamari')

        _, ssh_stdout, _ = ssh_client.exec_command(TAKE_PICS_CMD)
        ssh_stdout.channel.set_combine_stderr(True)

        for line in ssh_stdout:
            self.get_logger().info(line.strip())

        ftp_client = ssh_client.open_sftp()
        ftp_client.get(
            str(Path(REMOTE_PATH) / 'cam0.jpg'), str(Path(self.local_images_path) / 'cam0.jpg')
        )
        ftp_client.get(
            str(Path(REMOTE_PATH) / 'cam1.jpg'), str(Path(self.local_images_path) / 'cam1.jpg')
        )

        self.get_logger().info('Images downloaded from Calamari')

        img_1 = cv2.imread(str(Path(self.local_images_path) / 'cam0.jpg'))
        img_1 = cv2.rotate(img_1, cv2.ROTATE_180)
        img_msg_1 = self.cv_bridge.cv2_to_imgmsg(img_1)
        self.image_publisher_1.publish(img_msg_1)

        img_2 = cv2.imread(str(Path(self.local_images_path) / 'cam1.jpg'))
        img_2 = cv2.rotate(img_2, cv2.ROTATE_180)
        img_msg_2 = self.cv_bridge.cv2_to_imgmsg(img_2)
        self.image_publisher_2.publish(img_msg_2)

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
