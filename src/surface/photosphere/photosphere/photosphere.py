from rclpy.node import Node

from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_default
import pickle
import socket
import struct

import cv2

HOST_IP = '127.0.1.1' # the server ip address
PORT = 9997

class Photosphere(Node):
    def __init__(self) -> None:
        super().__init__('photosphere')

        self.fisheye1_publisher = self.create_publisher(
            Image, 'fisheye1_image', qos_profile_default
        )

        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((host_ip, port))
        data = b''
        payload_size = struct.calcsize('Q')

    def get_frame(self) -> None:

        while len(data) < payload_size:
            packet = client_socket.recv(4 * 1024) # 4K
            if not packet:
                break
            data += packet
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack('Q', packed_msg_size)[0]

        while len(data) < msg_size:
            data += client_socket.recv(4 * 1024)
        frame_data = data[:msg_size]
        data  = data[msg_size:]
        frame = pickle.loads(frame_data)

        time_msg = self.get_clock().now().to_msg()
        img_msg = self.get_image_msg(frame, time_msg)

        self.fisheye1_publisher.publish(img_msg)

        cv2.imshow('RECEIVING VIDEO', frame)

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
        client_socket.close()
        



def main() -> None:
    """Run the photosphere node."""
    rclpy.init()

    photosphere = Photosphere()
    executor = MultiThreadedExecutor()

    try:
        while True:
            rclpy.spin_once(photosphere, executor = executor, timeout_sec=0)
            photosphere.get_frame()
    finally:
        photosphere.shutdown()

if __name__ == '__main__':
    main()