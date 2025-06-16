from pathlib import Path

import cv2
import paramiko  # type: ignore[import-untyped]
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Image

from rov_msgs.srv import TakePhotosphere

HOST = '192.168.2.5'
USER = 'rov'
# Ruff complains about plaintext passwords
PASSWORD = 'rov12345'  # noqa: S105
CONNECT_TIMEOUT = 8  # Seconds to log in

# TAKE_PICS_CMD = 'bash /home/rov/rov-25/src/photosphere/take_images.sh'
TAKE_PICS_TEMPLATE = (
    'fswebcam -d /dev/video{video_num} -r 3840x3032 --skip 10 --scale 3840x3032 '
    '--no-banner /home/rov/rov-25/src/photosphere/images/cam{cam_num}.jpg'
)
TAKE_PICS_CMDS = {
    TakePhotosphere.Request.CAM0: TAKE_PICS_TEMPLATE.format(video_num=0, cam_num=0),
    TakePhotosphere.Request.CAM1: TAKE_PICS_TEMPLATE.format(video_num=2, cam_num=1),
    TakePhotosphere.Request.BOTH: TAKE_PICS_TEMPLATE.format(video_num=0, cam_num=0)
    + '\n'
    + TAKE_PICS_TEMPLATE.format(video_num=2, cam_num=1),
}

PIC_FILE_NAMES = {
    TakePhotosphere.Request.CAM0: ('cam0.jpg',),
    TakePhotosphere.Request.CAM1: ('cam1.jpg',),
    TakePhotosphere.Request.BOTH: ('cam0.jpg', 'cam1.jpg'),
}

LOCAL_PATH = 'src/surface/photosphere/images'  # relative the rov-25 repo
REMOTE_PATH = '/home/rov/rov-25/src/photosphere/images'


class PhotosphereDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('photosphere_driver_node')

        self.cv_bridge = CvBridge()

        self.arming_service = self.create_service(
            TakePhotosphere, 'take_photos', callback=self.take_photos_callback
        )

        self.image_publishers = {
            'cam0.jpg': self.create_publisher(Image, 'image_1', qos_profile_system_default),
            'cam1.jpg': self.create_publisher(Image, 'image_2', qos_profile_system_default),
        }

        self.local_images_path = (
            Path(get_package_share_directory('photosphere').split('rov-25')[0])
            / 'rov-25'
            / LOCAL_PATH
        )

        self.get_logger().info('Ready to download photosphere images')

    def take_photos_callback(
        self, request: TakePhotosphere.Request, response: TakePhotosphere.Response
    ) -> TakePhotosphere.Response:
        """Connect to the photosphere over ssh and take an image from each camera.

        Parameters
        ----------
        request : TakePhotosphere.Request
            The ROS service request (which camera(s))
        response : TakePhotosphere.Response
            The ROS service response (success)

        Returns
        -------
        TakePhotosphere.Response
            The filled ROS service response
        """
        ssh_client = paramiko.SSHClient()

        # Ruff complains about AutoAddPolicy being insecure
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())  # noqa: S507

        try:
            ssh_client.connect(HOST, username=USER, password=PASSWORD, timeout=CONNECT_TIMEOUT)
        except (TimeoutError, paramiko.ssh_exception.NoValidConnectionsError):
            self.get_logger().error('Failed to connect to photosphere sensor')
            response = TakePhotosphere.Response()
            response.success = False
            response.cam = request.cam
            return response

        self.get_logger().info('Connected to Calamari')

        _, ssh_stdout, _ = ssh_client.exec_command(TAKE_PICS_CMDS[request.cam])
        ssh_stdout.channel.set_combine_stderr(True)

        for line in ssh_stdout:
            self.get_logger().info(line.strip())

        ftp_client = ssh_client.open_sftp()

        for filename in PIC_FILE_NAMES[request.cam]:
            ftp_client.get(
                str(Path(REMOTE_PATH) / filename), str(Path(self.local_images_path) / filename)
            )

        self.get_logger().info('Images downloaded from Calamari')

        for filename in PIC_FILE_NAMES[request.cam]:
            img = cv2.imread(str(Path(self.local_images_path) / filename))
            img = cv2.rotate(img, cv2.ROTATE_180)
            img_msg = self.cv_bridge.cv2_to_imgmsg(img)
            self.image_publishers[filename].publish(img_msg)

        response = TakePhotosphere.Response()
        response.success = True
        response.cam = request.cam

        return response


def main() -> None:
    """Run the photosphere driver node."""
    rclpy.init()

    node = PhotosphereDriverNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
