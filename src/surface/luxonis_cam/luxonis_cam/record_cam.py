import os
import time

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Image

RECORD_FRAMERATE = 5


class RecordCamNode(Node):
    def __init__(self) -> None:
        super().__init__('record_cam_node', parameter_overrides=[])

        self.get_logger().info('Recording luxonis cam...')

        self.image_sub = self.create_subscription(
            Image, '/surface/lux_raw/image_raw', self.image_callback, qos_profile_system_default
        )

        self.cv_bridge = CvBridge()

        self.image_count = 0
        self.last_record_time: float | None = None

        self.image_dir = os.path.join(get_package_share_directory('luxonis_cam').split('rov-25')[0],
                                      'rov-25/src/surface/luxonis_cam/recorded_images')
        self.record_period = 1 / RECORD_FRAMERATE


    def image_callback(self, msg: Image) -> None:
        now = time.time()
        if self.last_record_time is not None and now - self.last_record_time < self.record_period:
            return

        self.get_logger().info('Recording image')

        cv2.imwrite(os.path.join(self.image_dir, f'{self.image_count:04}.jpg'),
                    self.cv_bridge.imgmsg_to_cv2(msg))

        if self.last_record_time is None:
            self.last_record_time = now
        else:
            self.last_record_time = max(self.last_record_time + self.record_period, now)
        self.image_count += 1


def main() -> None:
    rclpy.init()
    record_node = RecordCamNode()

    rclpy.spin(record_node)

if __name__ == '__main__':
    main()
