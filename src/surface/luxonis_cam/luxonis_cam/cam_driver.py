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
    tx_flag: bool

    @staticmethod
    def of(stream_name: str, topic: StreamTopic, *, tx_flag: bool) -> 'StreamMeta':
        return StreamMeta(topic=topic,
                          toggle_in_stream_name=f'{stream_name}_toggle_in',
                          script_toggle_name=f'{stream_name}_toggle',
                          script_input_name=f'{stream_name}_script_in',
                          script_output_name=f'{stream_name}_script_out',
                          out_stream_name=f'{stream_name}_out',
                          tx_flag=tx_flag)

# Alias for easier access to LUX_LEFT/LUX_RIGHT/etc.
CAM_IDS = CameraManage.Request

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

        if request.cam in self.stream_metas_dict:
            self.stream_metas_dict[request.cam].tx_flag = request.on
        else:
            response.success = False

        self.get_logger().info(f'TXing: {[meta.tx_flag for meta in self.stream_metas]}')

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

        self.stream_metas_dict = {
            CAM_IDS.LUX_LEFT: StreamMeta.of('left', StreamTopic.CAM0, tx_flag=True),
            CAM_IDS.LUX_RIGHT: StreamMeta.of('right', StreamTopic.CAM1, tx_flag=False),
            CAM_IDS.LUX_LEFT_RECT: StreamMeta.of('left_rect', StreamTopic.CAM0, tx_flag=False),
            CAM_IDS.LUX_RIGHT_RECT: StreamMeta.of('right_rect', StreamTopic.CAM1, tx_flag=False),
            CAM_IDS.LUX_DISPARITY: StreamMeta.of('disparity', StreamTopic.DISPARITY, tx_flag=False),
            CAM_IDS.LUX_DEPTH: StreamMeta.of('depth', StreamTopic.DEPTH, tx_flag=False)
        }

        self.stream_metas = tuple(self.stream_metas_dict.values())

        self.script = self.pipeline.createScript()
        script_str = \
f"""
tx_flags = [False] * {len(self.stream_metas)}
toggle_inputs = ["{'", "'.join([meta.script_toggle_name for meta in self.stream_metas])}"]
frame_inputs = ["{'", "'.join([meta.script_input_name for meta in self.stream_metas])}"]
frame_outputs = ["{'", "'.join([meta.script_output_name for meta in self.stream_metas])}"]

while True:
    for i, (toggle_input, frame_input, frame_output) in enumerate(zip(toggle_inputs, frame_inputs, frame_outputs)):
        toggle_msg = node.io[toggle_input].tryGet()
        if toggle_msg is not None:
            tx_flags[i] = toggle_msg.getData()[0]

        frame = node.io[frame_input].get()

        if tx_flags[i]:
            node.io[frame_output].send(frame)
"""
        self.get_logger().info('\nScript:\n"""' + script_str + '"""\n')
        self.script.setScript(script_str)

        for node, meta in zip(
            (self.left_cam_node, self.right_cam_node),
            [self.stream_metas_dict[cam_id] for cam_id in (CAM_IDS.LUX_LEFT, CAM_IDS.LUX_RIGHT)],
            strict=True
        ):
            # Camera frame reader -> script [script_input_name]
            node.setPreviewSize(640, 400)
            node.setInterleaved(False)
            node.setColorOrder(depthai.ColorCameraProperties.ColorOrder.RGB)
            node.preview.link(self.script.inputs[meta.script_input_name])

        self.stereo_node = self.pipeline.create(depthai.node.StereoDepth)
        self.stereo_node.setDefaultProfilePreset(depthai.node.StereoDepth.PresetMode.HIGH_ACCURACY)

        self.left_cam_node.isp.link(self.stereo_node.left)
        self.right_cam_node.isp.link(self.stereo_node.right)

        self.stereo_node.rectifiedLeft.link(self.script.inputs[self.stream_metas_dict[CAM_IDS.LUX_LEFT_RECT].script_input_name])
        self.stereo_node.rectifiedRight.link(self.script.inputs[self.stream_metas_dict[CAM_IDS.LUX_RIGHT_RECT].script_input_name])
        self.stereo_node.disparity.link(self.script.inputs[self.stream_metas_dict[CAM_IDS.LUX_DISPARITY].script_input_name])
        self.stereo_node.depth.link(self.script.inputs[self.stream_metas_dict[CAM_IDS.LUX_DEPTH].script_input_name])

        for stream_meta in self.stream_metas:
            # Camera toggler -> script [script_toggle_name]
            toggle_xin = self.pipeline.create(depthai.node.XLinkIn)
            toggle_xin.setStreamName(stream_meta.toggle_in_stream_name)
            toggle_xin.out.link(self.script.inputs[stream_meta.script_toggle_name])

            # script [script_output_name] -> cam_xout
            frame_xout = self.pipeline.create(depthai.node.XLinkOut)
            frame_xout.setStreamName(stream_meta.out_stream_name)
            frame_xout.input.setBlocking(False)
            frame_xout.input.setQueueSize(1)
            self.script.outputs[stream_meta.script_output_name].link(frame_xout.input)

        # Deploy pipeline to device
        while True:
            try:
                self.device = depthai.Device(self.pipeline).__enter__()
            except RuntimeError:
                self.get_logger().warning('Could not find Luxonis cam, retrying...')
                continue
            break

        self.toggle_queues = [self.device.getInputQueue(meta.toggle_in_stream_name) for meta in self.stream_metas]

        self.frame_output_queues = [self.device.getOutputQueue(stream.out_stream_name) for stream in self.stream_metas]

    def spin(self) -> None:
        for i, output_queue in enumerate(self.frame_output_queues):
            self.frame_publishers.try_get_publish(self.stream_metas[i].topic, output_queue)

        for i, toggle_queue in enumerate(self.toggle_queues):
            buf = depthai.Buffer()  # TODO: can we create this once and reuse?
            buf.setData(self.stream_metas[i].tx_flag)
            toggle_queue.send(buf)

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
