import os
import time
from collections.abc import Callable

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

        self.image_subs = (
            self.create_subscription(
                Image, '/surface/lux_raw/image_raw', self.make_image_callback('rect_right'), qos_profile_system_default
            ),
            self.create_subscription(
                Image, '/surface/lux_raw2/image_raw', self.make_image_callback('rect_left'), qos_profile_system_default
            ),
        )

        self.cv_bridge = CvBridge()

        self.image_count = 0
        self.last_record_times: dict[str, float | None] = {'rect_left': None, 'rect_right': None}

        self.image_dir = os.path.join(get_package_share_directory('luxonis_cam').split('rov-25')[0],
                                      'rov-25/src/surface/luxonis_cam/recorded_images')
        self.record_period = 1 / RECORD_FRAMERATE


    def make_image_callback(self, topic: str) -> Callable[[Image], None]:
        def image_callback(msg: Image) -> None:
            now = time.time()
            if self.last_record_times[topic] is not None and now - self.last_record_times[topic] < self.record_period:
                return

            self.get_logger().info(f'Recording {topic} image')

            file_path = os.path.join(self.image_dir, topic, f'{self.image_count:04}.{msg.header.stamp.sec}.{msg.header.stamp.nanosec}.jpg')
            cv2.imwrite(file_path, self.cv_bridge.imgmsg_to_cv2(msg))
            self.get_logger().info(file_path)

            if self.last_record_times[topic] is None:
                self.last_record_times[topic] = now
            else:
                self.last_record_times[topic] = max(self.last_record_times[topic] + self.record_period, now)
            self.image_count += 1
        return image_callback


def main() -> None:
    rclpy.init()
    record_node = RecordCamNode()

    rclpy.spin(record_node)

if __name__ == '__main__':
    main()
