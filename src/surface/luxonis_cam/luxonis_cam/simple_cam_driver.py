from enum import StrEnum

import cv2
import depthai
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from numpy import generic
from numpy.typing import NDArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image

from rov_msgs.msg import Intrinsics
from rov_msgs.srv import CameraManage

Matlike = NDArray[generic]


LEFT_CAM_SOCKET = depthai.CameraBoardSocket.CAM_A
RIGHT_CAM_SOCKET = depthai.CameraBoardSocket.CAM_D

MISSED_SENDS_RESET_THRESHOLD = 5

# FRAME_WIDTH = 1280
# FRAME_HEIGHT = 800
FRAME_WIDTH = 640
FRAME_HEIGHT = 400

# Alias for easier access to LUX_LEFT/LUX_RIGHT/etc.
CAM_IDS = CameraManage.Request


class StreamTopic(StrEnum):
    LUX_RAW = 'lux_raw/image_raw'
    RECT_LEFT = 'rect_left/image_raw'
    RECT_RIGHT = 'rect_right/image_raw'
    DISPARITY = 'disparity/image_raw'
    DEPTH = 'depth/image_raw'


class FramePublishers:
    """Singleton to manage publishing video frames."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self.publishers = {topic: self.make_frame_publisher(topic) for topic in StreamTopic}
        self.bridge = CvBridge()

    def make_frame_publisher(self, topic: StreamTopic) -> Publisher:
        """
        Create a publisher for the specified topic.

        Parameters
        ----------
        topic : StreamTopic
            the topic to publish on

        Returns
        -------
        Publisher
            the new publisher
        """
        return self.node.create_publisher(Image, topic.value, QoSPresetProfiles.DEFAULT.value)

    def try_get_publish(self, topic: StreamTopic, queue: depthai.DataOutputQueue) -> None:
        """
        Attempt to get a frame from the queue and publish it on the topic.

        Parameters
        ----------
        topic : StreamTopic
            topic to publish to
        queue : depthai.DataOutputQueue
            queue to read from (single read then give up, won't block long)
        """
        video_frame = queue.tryGet()

        # Discard None (failed to get frame)
        if video_frame is None:
            return

        # Type narrow to make mypy happy
        if not isinstance(video_frame, depthai.ImgFrame):
            self.node.get_logger().warn('Dequeued something other than an image frame, skipping')
            return

        time_msg = self.node.get_clock().now().to_msg()

        if video_frame is not None:
            img_msg = self.get_image_msg(video_frame.getCvFrame(), time_msg)
            if topic in self.publishers:
                self.publishers[topic].publish(img_msg)
            else:
                self.node.get_logger().warning(
                    f'Invalid camera publisher topic "{topic.value}", not publishing'
                )

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


class LuxonisCamDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('luxonis_cam_driver', parameter_overrides=[])

        self.cam_manage_service = self.create_service(
            CameraManage, 'manage_luxonis', self.cam_manage_callback
        )
        self.intrinsics_publishers = (
            self.create_publisher(
                Intrinsics, 'luxonis_left_intrinsics', QoSPresetProfiles.DEFAULT.value
            ),
            self.create_publisher(
                Intrinsics, 'luxonis_right_intrinsics', QoSPresetProfiles.DEFAULT.value
            ),
        )

        self.streaming_rectified = False

        self.deploy_pipeline()

        calib_data = self.device.readCalibration()
        focal_lengths_mm = [0.0, 0.0]
        self.intrinsics: list[list[list[float]]] = []
        try:
            for i, cam in enumerate((LEFT_CAM_SOCKET, RIGHT_CAM_SOCKET)):
                # 3um/px (https://docs.luxonis.com/hardware/sensors/OV9782)
                # / 1000 to get mm
                self.intrinsics.append(calib_data.getCameraIntrinsics(cam))
                focal_lengths_mm[i] = self.intrinsics[-1][0][0] * 3 / 1000
            self.get_logger().info(f'Focal lengths: {focal_lengths_mm}')
        except IndexError:
            self.get_logger().warn('Unable to get Luxonis intrinsics. Did you calibrate?')

        self.get_logger().info('Pipeline created')

        self.missed_sends = 0

        self.frame_publishers = FramePublishers(self)

        self.right_unrect_queue = self.device.getOutputQueue('right_eye')
        self.rect_left_queue = self.device.getOutputQueue('rect_left')
        self.rect_right_queue = self.device.getOutputQueue('rect_right')

    def deploy_pipeline(self) -> None:
        pipeline = depthai.Pipeline()

        # Ingest raw streams
        left_cam_node = pipeline.createColorCamera()
        left_cam_node.setBoardSocket(LEFT_CAM_SOCKET)
        left_cam_node.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_800_P)

        right_cam_node = pipeline.createColorCamera()
        right_cam_node.setBoardSocket(RIGHT_CAM_SOCKET)
        right_cam_node.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_800_P)
        right_cam_node.initialControl.setMisc('3a-follow', depthai.CameraBoardSocket.CAM_D.value)

        # Right cam unrect
        right_cam_node.setPreviewSize(FRAME_WIDTH, FRAME_HEIGHT)
        right_cam_node.setInterleaved(False)
        right_cam_node.setColorOrder(depthai.ColorCameraProperties.ColorOrder.RGB)

        xout = pipeline.create(depthai.node.XLinkOut)
        xout.setStreamName('right_eye')
        xout.input.setBlocking(False)
        xout.input.setQueueSize(1)

        right_cam_node.preview.link(xout.input)

        if self.streaming_rectified:
            # Left cam unrect
            left_cam_node.setPreviewSize(FRAME_WIDTH, FRAME_HEIGHT)
            left_cam_node.setInterleaved(False)
            left_cam_node.setColorOrder(depthai.ColorCameraProperties.ColorOrder.RGB)

            # Stereo
            stereo_node = pipeline.create(depthai.node.StereoDepth)
            stereo_node.setDefaultProfilePreset(depthai.node.StereoDepth.PresetMode.HIGH_DENSITY)
            left_cam_node.isp.link(stereo_node.left)
            right_cam_node.isp.link(stereo_node.right)

            for pair in (
                (stereo_node.rectifiedLeft, 'rect_left'),
                (stereo_node.rectifiedRight, 'rect_right'),
            ):
                frame_xout = pipeline.create(depthai.node.XLinkOut)
                frame_xout.setStreamName(pair[1])
                frame_xout.input.setBlocking(False)
                frame_xout.input.setQueueSize(1)
                pair[0].link(frame_xout.input)

        self.get_logger().info('Deploying pipeline...')

        # Deploy pipeline to device
        while True:
            try:
                self.device = depthai.Device(pipeline).__enter__()
            except RuntimeError as e:  # noqa: F841 (unused variable e for optional logging below)
                self.get_logger().warning(
                    'Error uploading to Luxonis cam, retrying '
                    '(see cam_driver to enable more details)...'
                )
                # Uncomment to get more details about errors
                # These are usually just "the cam is disconnected", but can be other things
                # self.get_logger().warning(str(e))
                continue
            break

    def cam_manage_callback(
        self, request: CameraManage.Request, response: CameraManage.Response
    ) -> CameraManage.Response:
        """
        Enable/disable streams based on cam manage service call.

        Parameters
        ----------
        request : CameraManage.Request
            CameraManage service request
        response : CameraManage.Response
            CameraManage service response template

        Returns
        -------
        CameraManage.Response
            the service response
        """
        response.success = True

        if request.cam in (CAM_IDS.LUX_LEFT_RECT, CAM_IDS.LUX_RIGHT_RECT):
            self.get_logger().info('Luxonis now publishing: Rectified')
            self.streaming_rectified = True
            self.deploy_pipeline()
        elif request.cam == CAM_IDS.LUX_RIGHT:
            self.get_logger().info('Luxonis now publishing: Raw')
            self.streaming_rectified = False
            self.deploy_pipeline()

        return response

    def spin(self) -> None:
        self.publish_intrinsics()

        try:
            self.frame_publishers.try_get_publish(StreamTopic.LUX_RAW, self.right_unrect_queue)

            if self.streaming_rectified:
                self.frame_publishers.try_get_publish(StreamTopic.RECT_LEFT, self.rect_left_queue)
                self.frame_publishers.try_get_publish(StreamTopic.RECT_RIGHT, self.rect_right_queue)
        except RuntimeError:
            self.missed_sends += 1
            self.get_logger().warn('Missed a dual cam spin')

        if self.missed_sends >= MISSED_SENDS_RESET_THRESHOLD:
            self.get_logger().error(
                f'Missed >= {MISSED_SENDS_RESET_THRESHOLD} dual cam spins, redeploying'
            )
            self.deploy_pipeline()
            self.missed_sends = 0

    def publish_intrinsics(self) -> None:
        if len(self.intrinsics) == len(self.intrinsics_publishers):
            # Only publish intrinsics if they've been set (cam is calibrated)
            for intrinsics, publisher in zip(
                self.intrinsics, self.intrinsics_publishers, strict=True
            ):
                publisher.publish(
                    Intrinsics(
                        fx=intrinsics[0][0],
                        fy=intrinsics[1][1],
                        x0=intrinsics[0][2],
                        y0=intrinsics[1][2],
                        s=intrinsics[0][1],
                    )
                )

    def shutdown(self) -> None:
        """Free the device and any other resources."""
        if self.device:
            self.device.close()


def main() -> None:
    rclpy.init()
    driver_node = LuxonisCamDriverNode()
    executor = MultiThreadedExecutor()

    try:
        while True:
            rclpy.spin_once(driver_node, executor=executor, timeout_sec=0)
            driver_node.spin()
    finally:
        driver_node.shutdown()
