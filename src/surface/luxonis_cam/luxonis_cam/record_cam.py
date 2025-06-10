import enum
from rov_msgs.srv import CameraManage

# CAM0_TOPIC = 'cam0/image_raw'
# CAM1_TOPIC = 'cam1/image_raw'

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
import cv2
import os

SAVE_PATH = "/home/rov/camera_recordings/luxonis_frames" 
os.makedirs(SAVE_PATH, exist_ok=True)

class RecordCam(Node):
    def __init__(self):
        super().__init__('record_cam')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image,
            'cam1/image_raw',  # <-- Replace with your topic if different
            self.listener_callback,
            qos
        )

        self.bridge = CvBridge()
        self.counter = 0

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            filename = os.path.join(SAVE_PATH, f"frame_{self.counter:06d}.jpg")
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Saved: {filename}")
            self.counter += 1
        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")

def main():
    rclpy.init()
    node = RecordCam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
