import subprocess
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from numpy import generic
from numpy.typing import NDArray
from rclpy.node import Node
from rclpy.qos import qos_profile_default
from sensor_msgs.msg import Image

from photosphere.fisheye_projection import convert_with_matrix
from rov_msgs.srv import GeneratePhotosphere

PROJECTION_PATH = 'src/photosphere/display/projection.jpg'  # relative the rov-25 repo
WEBSERVER_PATH = 'src/photosphere/display/'  # relative the rov-25 repo


Matlike = NDArray[generic]


class Photosphere(Node):
    def __init__(self) -> None:
        super().__init__('photosphere')

        self.cam_1_subscriber = self.create_subscription(
            Image, 'image_1', (lambda msg: self.handle_frame_msg(msg, 0)), qos_profile_default
        )

        self.cam_2_subscriber = self.create_subscription(
            Image, 'image_2', (lambda msg: self.handle_frame_msg(msg, 1)), qos_profile_default
        )

        self.fisheye_frames = [
            np.zeros((4000, 4000, 3), dtype=np.uint8),
            np.zeros((4000, 4000, 3), dtype=np.uint8),
        ]

        self.bridge = CvBridge()

        self.photosphere_service = self.create_service(
            GeneratePhotosphere, 'generate_photosphere', callback=self.photosphere_service_callback
        )

        subprocess.Popen(
            'python3 -m http.server -d '
            + (
                Path(get_package_share_directory('photosphere').split('rov-25')[0])
                / 'rov-25'
                / WEBSERVER_PATH
            ),
            shell=True,
        )

    def handle_frame_msg(self, msg: Image, index: int) -> None:
        """Handle a ros image message containing a frame from the photosphere.

        Parameters
        ----------
        msg : Image
            The ros image message
        index : int
            Which camera the image is from (0 or 1)
        """
        cv_img = self.bridge.imgmsg_to_cv2(msg)
        self.fisheye_frames[index] = cv_img

    def photosphere_service_callback(
        self, _: GeneratePhotosphere.Request, response: GeneratePhotosphere.Response
    ) -> GeneratePhotosphere.Response:
        """
        Handle a request to generate a photosphere.

        Parameters
        ----------
        request : GeneratePhotosphere.Request
            The request that caused this callback being called
        response : GeneratePhotosphere.Response
            The response to be sent

        Returns
        -------
        GeneratePhotosphere.Response
            The filled response
        """
        projection = convert_with_matrix(self.fisheye_frames[0], self.fisheye_frames[1])
        self.get_logger().info('Projection created')
        cv2.imwrite(
            (
                Path(get_package_share_directory('photosphere').split('rov-25')[0])
                / 'rov-25'
                / PROJECTION_PATH
            ),
            projection,
        )
        self.get_logger().info('Projection saved')

        response.generated = True
        return response


def main() -> None:
    """Run the photosphere node."""
    rclpy.init()

    photosphere = Photosphere()

    rclpy.spin(photosphere)


if __name__ == '__main__':
    main()
