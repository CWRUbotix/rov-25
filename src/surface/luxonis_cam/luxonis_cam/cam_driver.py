from dataclasses import dataclass
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

class StreamTopic(StrEnum):
    LUX_RAW = 'lux_raw/image_raw'
    RECT_LEFT = 'rect_left/image_raw'
    RECT_RIGHT = 'rect_right/image_raw'
    DISPARITY = 'disparity/image_raw'
    DEPTH = 'depth/image_raw'


@dataclass
class StreamScriptTopicSet:
    """Dataclass representing video stream script topics (toggle/frame I/O stream topics)."""

    toggle_in_stream_name: str
    script_toggle_name: str
    script_input_name: str
    script_output_name: str

    @staticmethod
    def of(stream_name: str) -> 'StreamScriptTopicSet':
        """
        Create a StreamScriptTopicSet (factory method).

        Parameters
        ----------
        stream_name : str
            name of the stream

        Returns
        -------
        StreamScriptNames
            a dataclass representing stream script topics
        """
        return StreamScriptTopicSet(
            toggle_in_stream_name=f'{stream_name}_toggle_in',
            script_toggle_name=f'{stream_name}_toggle',
            script_input_name=f'{stream_name}_script_in',
            script_output_name=f'{stream_name}_script_out',
        )


@dataclass
class StreamMeta:
    """Mutable dataclass representing video stream metadata."""

    topic: StreamTopic
    script_topics: StreamScriptTopicSet
    out_stream_name: str
    enabled: bool

    @staticmethod
    def of(stream_name: str, topic: StreamTopic, *, enabled: bool) -> 'StreamMeta':
        """
        Create a StreamMeta (factory method).

        Parameters
        ----------
        stream_name : str
            name of the stream
        topic : StreamTopic
            ROS topic the stream will be published on
        enabled : bool
            whether the stream is enabled by default

        Returns
        -------
        StreamMeta
            a mutable dataclass representing stream metadata
        """
        return StreamMeta(
            topic=topic,
            script_topics=StreamScriptTopicSet.of(stream_name),
            out_stream_name=f'{stream_name}_out',
            enabled=enabled,
        )


# Alias for easier access to LUX_LEFT/LUX_RIGHT/etc.
CAM_IDS = CameraManage.Request


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


STREAMS_THAT_NEED_STEREO = [
    CAM_IDS.LUX_LEFT_RECT,
    CAM_IDS.LUX_RIGHT_RECT,
    CAM_IDS.LUX_DISPARITY,
    CAM_IDS.LUX_DEPTH,
]


class LuxonisCamDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('luxonis_cam_driver', parameter_overrides=[])

        self.stream_metas = {
            CAM_IDS.LUX_LEFT: StreamMeta.of('left', StreamTopic.LUX_RAW, enabled=False),
            CAM_IDS.LUX_RIGHT: StreamMeta.of('right', StreamTopic.LUX_RAW, enabled=False),
            CAM_IDS.LUX_LEFT_RECT: StreamMeta.of('left_rect', StreamTopic.RECT_LEFT, enabled=False),
            CAM_IDS.LUX_RIGHT_RECT: StreamMeta.of(
                'right_rect', StreamTopic.RECT_RIGHT, enabled=False
            ),
            CAM_IDS.LUX_DISPARITY: StreamMeta.of('disparity', StreamTopic.DISPARITY, enabled=False),
            CAM_IDS.LUX_DEPTH: StreamMeta.of('depth', StreamTopic.DEPTH, enabled=False),
        }

        self.left_stereo_script_topics = StreamScriptTopicSet.of('left_stereo')
        self.right_stereo_script_topics = StreamScriptTopicSet.of('right_stereo')
        self.script_topics = (
            *(meta.script_topics for meta in self.stream_metas.values()),
            self.left_stereo_script_topics,
            self.right_stereo_script_topics,
        )

        self.cam_manage_service = self.create_service(
            CameraManage, 'manage_luxonis', self.cam_manage_callback
        )
        self.intrinsics_publishers = (
            self.create_publisher(
                Intrinsics, 'luxonis_left_intrinsics', QoSPresetProfiles.DEFAULT.value
            ),
            self.create_publisher(
                Intrinsics, 'luxonis_right_intrinsics', QoSPresetProfiles.DEFAULT.value
            )
        )

        self.create_pipeline()

        self.frame_publishers = FramePublishers(self)

        self.get_logger().info('Pipeline created')

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

        if request.cam in self.stream_metas:
            self.stream_metas[request.cam].enabled = request.on
        else:
            response.success = False

        statuses = [f'{cam}: {meta.enabled}' for cam, meta in self.stream_metas.items()]
        self.get_logger().info(f'Luxonis now publishing: {"; ".join(statuses)}')

        return response

    def create_pipeline(self) -> None:
        """Create a depthai pipeline and deploy it to the camera."""
        pipeline = depthai.Pipeline()

        left_cam_node = pipeline.createColorCamera()
        left_cam_node.setBoardSocket(LEFT_CAM_SOCKET)
        left_cam_node.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_800_P)

        right_cam_node = pipeline.createColorCamera()
        right_cam_node.setBoardSocket(RIGHT_CAM_SOCKET)
        right_cam_node.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_800_P)
        right_cam_node.initialControl.setMisc('3a-follow', depthai.CameraBoardSocket.CAM_D.value)

        script = pipeline.createScript()
        script_str = f"""
enabled_flags = [False] * {len(self.script_topics)}
toggle_inputs = ["{'", "'.join([names.script_toggle_name for names in self.script_topics])}"]
frame_inputs = ["{'", "'.join([names.script_input_name for names in self.script_topics])}"]
frame_outputs = ["{'", "'.join([names.script_output_name for names in self.script_topics])}"]

while True:
    for i, (toggle_input, frame_input, frame_output) in enumerate(zip(toggle_inputs, frame_inputs,
                                                                      frame_outputs)):
        toggle_msg = node.io[toggle_input].tryGet()
        if toggle_msg is not None:
            enabled_flags[i] = toggle_msg.getData()[0]

        frame = node.io[frame_input].tryGet()

        if frame is not None and enabled_flags[i]:
            node.io[frame_output].send(frame)
"""
        self.get_logger().info('\nScript:\n"""' + script_str + '"""\n')
        script.setScript(script_str)

        for node, meta in zip(
            (left_cam_node, right_cam_node),
            [self.stream_metas[cam_id] for cam_id in (CAM_IDS.LUX_LEFT, CAM_IDS.LUX_RIGHT)],
            strict=True,
        ):
            # Camera frame reader -> script [script_input_name]
            node.setPreviewSize(640, 400)
            node.setInterleaved(False)
            node.setColorOrder(depthai.ColorCameraProperties.ColorOrder.RGB)
            node.preview.link(script.inputs[meta.script_topics.script_input_name])

        stereo_node = pipeline.create(depthai.node.StereoDepth)
        stereo_node.setDefaultProfilePreset(depthai.node.StereoDepth.PresetMode.HIGH_DENSITY)

        for names in self.script_topics:
            # Camera toggler -> script [script_toggle_name]
            toggle_xin = pipeline.create(depthai.node.XLinkIn)
            toggle_xin.setStreamName(names.toggle_in_stream_name)
            toggle_xin.setMaxDataSize(1)
            toggle_xin.out.link(script.inputs[names.script_toggle_name])

        left_cam_node.isp.link(script.inputs[self.left_stereo_script_topics.script_input_name])
        right_cam_node.isp.link(script.inputs[self.right_stereo_script_topics.script_input_name])
        script.outputs[self.left_stereo_script_topics.script_output_name].link(stereo_node.left)
        script.outputs[self.right_stereo_script_topics.script_output_name].link(stereo_node.right)

        stereo_node.rectifiedLeft.link(
            script.inputs[self.stream_metas[CAM_IDS.LUX_LEFT_RECT].script_topics.script_input_name]
        )
        stereo_node.rectifiedRight.link(
            script.inputs[self.stream_metas[CAM_IDS.LUX_RIGHT_RECT].script_topics.script_input_name]
        )
        stereo_node.disparity.link(
            script.inputs[self.stream_metas[CAM_IDS.LUX_DISPARITY].script_topics.script_input_name]
        )
        stereo_node.depth.link(
            script.inputs[self.stream_metas[CAM_IDS.LUX_DEPTH].script_topics.script_input_name]
        )

        for stream_meta in self.stream_metas.values():
            # script [script_output_name] -> cam_xout
            frame_xout = pipeline.create(depthai.node.XLinkOut)
            frame_xout.setStreamName(stream_meta.out_stream_name)
            frame_xout.input.setBlocking(False)
            frame_xout.input.setQueueSize(1)
            script.outputs[stream_meta.script_topics.script_output_name].link(frame_xout.input)

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
                # self.get_logger().warning(e)
                continue
            break

        self.left_stereo_toggle_queue = self.device.getInputQueue('left_stereo_toggle_in')
        self.right_stereo_toggle_queue = self.device.getInputQueue('right_stereo_toggle_in')
        self.toggle_queues = {
            cam_id: self.device.getInputQueue(meta.script_topics.toggle_in_stream_name)
            for cam_id, meta in self.stream_metas.items()
        }
        self.frame_output_queues = {
            cam_id: self.device.getOutputQueue(meta.out_stream_name)
            for cam_id, meta in self.stream_metas.items()
        }

        self.get_logger().info('Pipeline deployed')

        calib_data = self.device.readCalibration()
        focal_lengths_mm = [0.0, 0.0]
        self.intrinsics: list[list[list[float]]] = []
        for i, cam in enumerate((LEFT_CAM_SOCKET, RIGHT_CAM_SOCKET)):
            # 3um/px (https://docs.luxonis.com/hardware/sensors/OV9782)
            # / 1000 to get mm
            self.intrinsics.append(calib_data.getCameraIntrinsics(cam))
            focal_lengths_mm[i] = self.intrinsics[-1][0][0] * 3 / 1000
        self.get_logger().info(f'focal lengths: {focal_lengths_mm}')

        # intrinsics = calib_data.getCameraIntrinsics(depthai.CameraBoardSocket.CAM_A)
        # self.get_logger().info(f'CAM_A focal length in pixels: {intrinsics[0][0]}')
        # # self.get_logger().info(f'CAM_A intrinsics: {intrinsics}')

        # intrinsics = calib_data.getCameraIntrinsics(depthai.CameraBoardSocket.CAM_D)
        # self.get_logger().info(f'CAM_D focal length in pixels: {intrinsics[0][0]}')
        # # self.get_logger().info(f'CAM_D intrinsics: {intrinsics}')

    def spin(self) -> None:
        """Run one iteration of I/O with the Luxonis cam."""
        for intrinsics, publisher in zip(self.intrinsics, self.intrinsics_publishers, strict=True):
            publisher.publish(
                Intrinsics(
                    fx=intrinsics[0][0],
                    fy=intrinsics[1][1],
                    x0=intrinsics[0][2],
                    y0=intrinsics[1][2],
                    s=intrinsics[0][1]
                )
            )

        # TODO: only send toggles when we actually need to change state?
        for cam_id, output_queue in self.frame_output_queues.items():
            if self.stream_metas[cam_id].enabled:
                self.frame_publishers.try_get_publish(self.stream_metas[cam_id].topic, output_queue)

        enable_stereo = False
        for cam_id in STREAMS_THAT_NEED_STEREO:
            if self.stream_metas[cam_id].enabled:
                enable_stereo = True
                break

        buf = depthai.Buffer()  # TODO: can we create this once and reuse?
        buf.setData([1 if enable_stereo else 0])
        self.left_stereo_toggle_queue.send(buf)
        self.right_stereo_toggle_queue.send(buf)

        for cam_id, toggle_queue in self.toggle_queues.items():
            buf = depthai.Buffer()
            buf.setData([1 if self.stream_metas[cam_id].enabled else 0])
            toggle_queue.send(buf)

        # disparity_frame = self.disparity_queue.tryGet()

        # if disparity_frame:
        #     frame = disparity_frame.getFrame()
        #     frame = (frame * (255 / stereo_node.initialConfig.getMaxDisparity())).astype(uint8)

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
