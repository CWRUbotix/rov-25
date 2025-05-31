from photosphere.fisheye_projection import convert_with_matrix
from rclpy.node import Node

from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_default
from builtin_interfaces.msg import Time
from numpy.typing import NDArray
from numpy import generic, uint8
from cv_bridge import CvBridge
import pickle
import socket
import struct
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rov_msgs.srv import GeneratePhotosphere
import numpy as np

import cv2

HOST_IP = '127.0.1.1' # the server ip address
PORT1 = 9997
PORT2 = 9998

Matlike = NDArray[generic]

class Photosphere(Node):
    def __init__(self) -> None:
        super().__init__('photosphere')

        # self.fisheye1_publisher = self.create_publisher(
        #     Image, 'fisheye1_image', qos_profile_default
        # )

        # self.fisheye2_publisher = self.create_publisher(
        #     Image, 'fisheye2_image', qos_profile_default
        # )

        self.fisheye_publishers = (
            self.create_publisher(
                Image, 'fisheye1_image', qos_profile_default
            ),
            self.create_publisher(
                Image, 'fisheye2_image', qos_profile_default
            )
        )

        self.client_sockets = (
            socket.socket(socket.AF_INET, socket.SOCK_STREAM),
            socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        )

        self.fisheye_frames = [
            np.zeros((4000, 4000, 3), dtype=np.uint8),
            np.zeros((4000, 4000, 3), dtype=np.uint8)
        ]

        # self.client_socket1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.client_socket1.connect((HOST_IP, PORT1))

        self.client_sockets[0].connect((HOST_IP, PORT1))
        self.client_sockets[1].connect((HOST_IP, PORT2))

        self.data = [b'', b'']
        self.payload_size = struct.calcsize('Q')
        self.bridge = CvBridge()

        self.photosphere_service = self.create_service(GeneratePhotosphere, 'generate_photosphere', callback = self.photosphere_service_callback)

    def get_frame(self, img_num: int) -> None:

        while len(self.data[img_num]) < self.payload_size:
            packet = self.client_sockets[img_num].recv(4 * 1024) # 4K
            if not packet:
                break
            self.data[img_num] += packet
        packed_msg_size = self.data[img_num][:self.payload_size]
        self.data[img_num] = self.data[img_num][self.payload_size:]
        msg_size = struct.unpack('Q', packed_msg_size)[0]

        while len(self.data[img_num]) < msg_size:
            self.data[img_num] += self.client_sockets[img_num].recv(4 * 1024)
        frame_data = self.data[img_num][:msg_size]
        self.data[img_num]  = self.data[img_num][msg_size:]
        frame = pickle.loads(frame_data)

        time_msg = self.get_clock().now().to_msg()
        img_msg = self.get_image_msg(frame, time_msg)

        self.fisheye_frames[img_num] = frame

        self.fisheye_publishers[img_num].publish(img_msg)

        # cv2.imshow('RECEIVING VIDEO', frame)

    def get_image_msg(self, image: Matlike, time: Time) -> Image:
        """Convert cv2 image to ROS2 Image with CvBridge.
        Parameters
        ----------
        image : Matlike
            The image to convert
        time : Time
            The timestamp for the ros message
        Returns
        -------
        Image
            The ROS2 image message
        """
        inverted_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img_msg: Image = self.bridge.cv2_to_imgmsg(inverted_image)
        img_msg.encoding = 'rgb8'
        img_msg.header.stamp = time
        return img_msg

    def shutdown(self) -> None:
        self.client_sockets[0].close()
        self.client_sockets[1].close()
    
    def photosphere_service_callback(self, request: GeneratePhotosphere.Request, response: GeneratePhotosphere.Response) -> GeneratePhotosphere.Response:
        """
        Handle a request to generate a photosphere

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
        cv2.imwrite('src/surface/photosphere/photosphere/frame1.png', self.fisheye_frames[0])
        cv2.imwrite('src/surface/photosphere/photosphere/frame2.png', self.fisheye_frames[1])
        print("frames saved")
        projection = convert_with_matrix(self.fisheye_frames[0], self.fisheye_frames[1])
        print("projection created")
        cv2.imwrite('src/surface/photosphere/photosphere/projection.png', projection)
        print("projection saved")
        # fisheye_image1 = cv2.imread('src/surface/photosphere/photosphere/frame1.png')
        # fisheye_image2 = cv2.imread('src/surface/photosphere/photosphere/frame2.png')
        # projection = equirectangular_projection(fisheye_image1, fisheye_image2)
        # cv2.imwrite('src/surface/photosphere/photosphere/projection.png', projection)

        response.generated = True
        return response
        



def main() -> None:
    """Run the photosphere node."""
    rclpy.init()

    photosphere = Photosphere()
    executor = MultiThreadedExecutor()

    try:
        while True:
            rclpy.spin_once(photosphere, executor = executor, timeout_sec=0)
            photosphere.get_frame(0)
            photosphere.get_frame(1)
    finally:
        photosphere.shutdown()

if __name__ == '__main__':
    main()