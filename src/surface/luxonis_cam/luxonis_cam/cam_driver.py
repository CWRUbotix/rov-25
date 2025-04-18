import cv2
import depthai
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from numpy import generic, uint8
from numpy.typing import NDArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image

Matlike = NDArray[generic]


class LuxonisCamDriverNode(Node):
    # CAM_TO_STREAM = 'left'  # which cam to stream to ros, 'left' or 'right'

    def __init__(self) -> None:
        super().__init__('luxonis_cam_driver', parameter_overrides=[])

        self.cam_to_stream_param = self.declare_parameter('cam_to_stream', '')
        self.cam_to_stream = self.cam_to_stream_param.value

        self.bridge = CvBridge()

        self.video_publisher = self.create_publisher(
            Image,
            f'luxonis_cam_stream_{self.cam_to_stream}',
            QoSPresetProfiles.DEFAULT.value,
        )

        self.create_pipeline()

        self.get_logger().info('Pipeline created')

    def create_pipeline(self) -> None:
        """Create a depthai pipeline and deploys it to the camera."""
        self.pipeline = depthai.Pipeline()

        self.left_cam_node = self.pipeline.createColorCamera()
        self.left_cam_node.setBoardSocket(depthai.CameraBoardSocket.CAM_D)
        self.left_cam_node.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_800_P)

        self.right_cam_node = self.pipeline.createColorCamera()
        self.right_cam_node.setBoardSocket(depthai.CameraBoardSocket.CAM_A)
        self.right_cam_node.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_800_P)
        self.right_cam_node.initialControl.setMisc('3a-follow', depthai.CameraBoardSocket.CAM_D)

        self.cam_to_stream_node: depthai.node.ColorCamera | None = None
        # TODO: Launching 2 copies of this node seems to not work
        # We could destroy/recreate pipelines whenever we want to change what we're streaming
        # but Benjamin thinks this might be better: https://docs.luxonis.com/software/depthai/examples/script_change_pipeline_flow
        if self.cam_to_stream == 'left':
            self.cam_to_stream_node = self.left_cam_node
        elif self.cam_to_stream == 'right':
            self.cam_to_stream_node = self.right_cam_node

        if self.cam_to_stream_node:
            self.cam_to_stream_node.setPreviewSize(640, 400)
            self.cam_to_stream_node.setInterleaved(False)
            self.cam_to_stream_node.setColorOrder(depthai.ColorCameraProperties.ColorOrder.RGB)

            self.preview_out_node = self.pipeline.create(depthai.node.XLinkOut)
            self.preview_out_node.setStreamName('preview')
            self.preview_out_node.input.setBlocking(False)
            self.preview_out_node.input.setQueueSize(1)

            self.cam_to_stream_node.preview.link(self.preview_out_node.input)

        # Setup stereo pipeline
        self.stereo_node = self.pipeline.create(depthai.node.StereoDepth)
        self.stereo_node.setDefaultProfilePreset(depthai.node.StereoDepth.PresetMode.HIGH_ACCURACY)

        self.left_cam_node.isp.link(self.stereo_node.left)
        self.right_cam_node.isp.link(self.stereo_node.right)

        self.disparity_out_node = self.pipeline.create(depthai.node.XLinkOut)
        self.disparity_out_node.setStreamName('disparity')
        self.stereo_node.disparity.link(self.disparity_out_node.input)

        # Deploy pipeline to device
        while True:
            try:
                self.device = depthai.Device(self.pipeline).__enter__()
            except RuntimeError:
                self.get_logger().warning('Could not find Luxonis cam, retrying...')
                continue
            break

        if self.cam_to_stream_node:
            self.video_queue = self.device.getOutputQueue('preview', maxSize=1, blocking=False)

        self.disparity_queue = self.device.getOutputQueue('disparity', maxSize=1, blocking=False)

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

    def spin(self) -> None:
        if self.cam_to_stream_node:
            video_frame = self.video_queue.tryGet()

            time_msg = self.get_clock().now().to_msg()

            if video_frame:
                img_msg = self.get_image_msg(video_frame.getCvFrame(), time_msg)
                self.video_publisher.publish(img_msg)

        disparity_frame = self.disparity_queue.tryGet()  # blocking call, will wait until a new data has arrived

        if disparity_frame:
            frame = disparity_frame.getFrame()
            frame = (frame * (255 / self.stereo_node.initialConfig.getMaxDisparity())).astype(uint8)

            cv2.imshow('disparity', frame)
            frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            cv2.imshow('disparity_color', frame)

            if cv2.waitKey(1) == ord('q'):
                raise KeyboardInterrupt

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
