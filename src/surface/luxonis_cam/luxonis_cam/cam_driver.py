from dataclasses import dataclass
from enum import StrEnum

import cv2
import depthai
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from depthai.node import ColorCamera
from numpy import generic, uint8
from numpy.typing import NDArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node, Publisher
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image

from rov_msgs.srv import CameraManage

"""
Cam -> topic plan:
 - unrectified_left -> cam0
 - unrectified_right -> cam1
 - rectified_left -> cam0
 - rectified_right -> cam1
 - disparity -> disparity
 - depth -> depth
"""

Matlike = NDArray[generic]

class StreamTopic(StrEnum):
    CAM0 = 'cam0_stream'
    CAM1 = 'cam1_stream'
    DISPARITY = 'disparity_stream'
    DEPTH = 'depth_stream'

@dataclass
class StreamMeta:
    topic: StreamTopic
    toggle_in_stream_name: str
    script_toggle_name: str
    script_input_name: str
    script_output_name: str
    out_stream_name: str

    @staticmethod
    def of(stream_name: str, topic: StreamTopic) -> 'StreamMeta':
        return StreamMeta(topic=topic,
                          toggle_in_stream_name=f'{stream_name}_toggle_in',
                          script_toggle_name=f'{stream_name}_toggle',
                          script_input_name=f'{stream_name}_script_in',
                          script_output_name=f'{stream_name}_script_out',
                          out_stream_name=f'{stream_name}_out')

@dataclass
class CamMeta:
    node: ColorCamera
    stream_meta: StreamMeta

class FramePublishers:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.publishers = {topic: self.make_frame_publisher(topic) for topic in StreamTopic}
        self.bridge = CvBridge()

    def make_frame_publisher(self, topic: StreamTopic) -> Publisher:
        return self.node.create_publisher(Image, topic.value, QoSPresetProfiles.DEFAULT.value)

    def try_get_publish(self, topic: StreamTopic, queue: depthai.DataOutputQueue) -> None:
        video_frame = queue.tryGet()
        time_msg = self.node.get_clock().now().to_msg()

        if video_frame is not None:
            img_msg = self.get_image_msg(video_frame.getCvFrame(), time_msg)
            if topic in self.publishers:
                self.publishers[topic].publish(img_msg)
            else:
                self.node.get_logger().warning(f'Invalid camera publisher topic "{topic.value}", '
                                               'not publishing')

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

        self.create_pipeline()

        self.frame_publishers = FramePublishers(self)

        self.get_logger().info('Pipeline created')

    def cam_manage_callback(
        self, request: CameraManage.Request, response: CameraManage.Response
    ) -> CameraManage.Response:
        response.success = True

        if request.cam == CameraManage.Request.LUX_LEFT:
            self.tx_left = request.on
        elif request.cam == CameraManage.Request.LUX_RIGHT:
            self.tx_right = request.on
        else:
            response.success = False

        self.get_logger().info(f'TXing: ({self.tx_left}, {self.tx_right})')

        return response

    def create_pipeline(self) -> None:
        """Create a depthai pipeline and deploys it to the camera."""
        self.pipeline = depthai.Pipeline()

        self.left_cam_node = self.pipeline.createColorCamera()
        self.left_cam_node.setBoardSocket(depthai.CameraBoardSocket.CAM_D)
        self.left_cam_node.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_800_P)

        self.right_cam_node = self.pipeline.createColorCamera()
        self.right_cam_node.setBoardSocket(depthai.CameraBoardSocket.CAM_A)
        self.right_cam_node.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_800_P)
        # self.right_cam_node.initialControl.setMisc('3a-follow', depthai.CameraBoardSocket.CAM_D)

        self.stream_metas = (
            StreamMeta.of('left', StreamTopic.CAM0),
            StreamMeta.of('right', StreamTopic.CAM1),
            StreamMeta.of('disparity', StreamTopic.DISPARITY)
        )

        self.cam_metas = (
            CamMeta(self.left_cam_node, self.stream_metas[0]),
            CamMeta(self.right_cam_node, self.stream_metas[1])
        )

        self.script = self.pipeline.createScript()
        self.script.setScript(
f"""
tx_left = False
tx_right = False

while True:
    left_toggle_msg = node.io['{self.cam_metas[0].stream_meta.script_toggle_name}'].tryGet()
    right_toggle_msg = node.io['{self.cam_metas[1].stream_meta.script_toggle_name}'].tryGet()

    if left_toggle_msg is not None:
        tx_left = left_toggle_msg.getData()[0]
    if right_toggle_msg is not None:
        tx_right = right_toggle_msg.getData()[0]

    left_frame = node.io['{self.cam_metas[0].stream_meta.script_input_name}'].get()
    right_frame = node.io['{self.cam_metas[1].stream_meta.script_input_name}'].get()

    if tx_left:
        node.io['{self.cam_metas[0].stream_meta.script_output_name}'].send(left_frame)
    if tx_right:
        node.io['{self.cam_metas[1].stream_meta.script_output_name}'].send(right_frame)
""")


        # NOPE! cam_meta.node -> cam_xout(cam_meta.out_stream_name)
        # cam_meta.node -> script.inputs[cam_meta.toggle_input_name]
        # toggle_xin(cam_meta.in_stream_name) -> script.inputs[cam_meta.toggle_input_name]

        for cam_meta in self.cam_metas:
            # Camera frame reader -> script [script_input_name]
            cam_meta.node.setPreviewSize(640, 400)
            cam_meta.node.setInterleaved(False)
            cam_meta.node.setColorOrder(depthai.ColorCameraProperties.ColorOrder.RGB)
            cam_meta.node.preview.link(self.script.inputs[cam_meta.stream_meta.script_input_name])

            # Camera toggler -> script [script_toggle_name]
            toggle_xin = self.pipeline.create(depthai.node.XLinkIn)
            toggle_xin.setStreamName(cam_meta.stream_meta.toggle_in_stream_name)
            toggle_xin.out.link(self.script.inputs[cam_meta.stream_meta.script_toggle_name])

            # script [script_output_name] -> cam_xout
            cam_xout = self.pipeline.create(depthai.node.XLinkOut)
            cam_xout.setStreamName(cam_meta.stream_meta.out_stream_name)
            cam_xout.input.setBlocking(False)
            cam_xout.input.setQueueSize(1)
            self.script.outputs[cam_meta.stream_meta.script_output_name].link(cam_xout.input)

        # Setup stereo pipeline
        # self.stereo_node = self.pipeline.create(depthai.node.StereoDepth)
        # self.stereo_node.setDefaultProfilePreset(depthai.node.StereoDepth.PresetMode.HIGH_ACCURACY)

        # self.left_cam_node.isp.link(self.stereo_node.left)
        # self.right_cam_node.isp.link(self.stereo_node.right)

        # self.disparity_out_node = self.pipeline.create(depthai.node.XLinkOut)
        # self.disparity_out_node.setStreamName('disparity_out')
        # self.stereo_node.disparity.link(self.disparity_out_node.input)

        # Deploy pipeline to device
        while True:
            try:
                self.device = depthai.Device(self.pipeline).__enter__()
            except RuntimeError:
                self.get_logger().warning('Could not find Luxonis cam, retrying...')
                continue
            break

        # if self.cam_to_stream_node:
        #     self.video_queue = self.device.getOutputQueue('preview', maxSize=1, blocking=False)

        # self.disparity_queue = self.device.getOutputQueue('disparity', maxSize=1, blocking=False)

        self.left_toggle_queue = self.device.getInputQueue(self.cam_metas[0].stream_meta.toggle_in_stream_name)
        self.right_toggle_queue = self.device.getInputQueue(self.cam_metas[1].stream_meta.toggle_in_stream_name)

        self.output_video_queues = [(self.device.getOutputQueue(stream.out_stream_name), stream.topic)
                                    for stream in self.stream_metas]

        self.tx_left = False
        self.tx_right = True

    def spin(self) -> None:
        for queue, topic in self.output_video_queues:
            self.frame_publishers.try_get_publish(topic, queue)

        buf = depthai.Buffer()
        buf.setData(self.tx_left)
        self.left_toggle_queue.send(buf)

        buf = depthai.Buffer()
        buf.setData(self.tx_right)
        self.right_toggle_queue.send(buf)

        # disparity_frame = self.disparity_queue.tryGet()  # blocking call, will wait until a new data has arrived

        # if disparity_frame:
        #     frame = disparity_frame.getFrame()
        #     frame = (frame * (255 / self.stereo_node.initialConfig.getMaxDisparity())).astype(uint8)

        #     cv2.imshow('disparity', frame)
        #     frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
        #     cv2.imshow('disparity_color', frame)

        #     if cv2.waitKey(1) == ord('q'):
        #         raise KeyboardInterrupt

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
