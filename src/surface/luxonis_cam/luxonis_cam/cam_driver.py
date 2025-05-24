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
import rclpy.logging
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
    CAM0 = 'cam0/image_raw'
    LUX_RAW = 'lux_raw/image_raw'
    RECT_LEFT = 'rect_left/image_raw'
    RECT_RIGHT = 'rect_right/image_raw'
    DISPARITY = 'disparity/image_raw'
    DEPTH = 'depth/image_raw'

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
    
STREAMS_THAT_NEED_STEREO = [CAM_IDS.LUX_LEFT_RECT, CAM_IDS.LUX_RIGHT_RECT]#, CAM_IDS.LUX_DISPARITY, CAM_IDS.LUX_DEPTH]

class LuxonisCamDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('luxonis_cam_driver', parameter_overrides=[])

        self.stream_metas_dict = {
            CAM_IDS.LUX_LEFT: StreamMeta.of('left', StreamTopic.LUX_RAW, tx_flag=False),
            CAM_IDS.LUX_RIGHT: StreamMeta.of('right', StreamTopic.LUX_RAW, tx_flag=False),
            CAM_IDS.LUX_LEFT_RECT: StreamMeta.of('left_rect', StreamTopic.RECT_LEFT, tx_flag=False),
            CAM_IDS.LUX_RIGHT_RECT: StreamMeta.of('right_rect', StreamTopic.RECT_RIGHT, tx_flag=False),
            # CAM_IDS.LUX_DISPARITY: StreamMeta.of('disparity', StreamTopic.DISPARITY, tx_flag=False),
            # CAM_IDS.LUX_DEPTH: StreamMeta.of('depth', StreamTopic.DEPTH, tx_flag=False)
        }

        self.stream_metas = tuple(self.stream_metas_dict.values())

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

        self.get_logger().info('CAM MANAGE CALLBACK')

        if request.cam in self.stream_metas_dict:
            self.stream_metas_dict[request.cam].tx_flag = request.on
        else:
            response.success = False

        self.get_logger().info(f'TXing: {[meta.tx_flag for meta in self.stream_metas]}')

        return response

    def create_pipeline(self) -> None:
        """Create a depthai pipeline and deploys it to the camera."""
        pipeline = depthai.Pipeline()

        left_cam_node = pipeline.createColorCamera()
        left_cam_node.setBoardSocket(depthai.CameraBoardSocket.CAM_D)
        left_cam_node.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_800_P)

        right_cam_node = pipeline.createColorCamera()
        right_cam_node.setBoardSocket(depthai.CameraBoardSocket.CAM_A)
        right_cam_node.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_800_P)
        # right_cam_node.initialControl.setMisc('3a-follow', depthai.CameraBoardSocket.CAM_D)

        script = pipeline.createScript()
        script_str = \
f"""
tx_flags = [False] * {len(self.stream_metas) + 2}
toggle_inputs = ["{'", "'.join([meta.script_toggle_name for meta in self.stream_metas])}", "left_stereo_toggle", "right_stereo_toggle"]
frame_inputs = ["{'", "'.join([meta.script_input_name for meta in self.stream_metas])}", "left_stereo_script_in", "right_stereo_script_in"]
frame_outputs = ["{'", "'.join([meta.script_output_name for meta in self.stream_metas])}", "left_stereo_script_out", "right_stereo_script_out"]

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
        script.setScript(script_str)

        for node, meta in zip(
            (left_cam_node, right_cam_node),
            [self.stream_metas_dict[cam_id] for cam_id in (CAM_IDS.LUX_LEFT, CAM_IDS.LUX_RIGHT)],
            strict=True
        ):
            # Camera frame reader -> script [script_input_name]
            node.setPreviewSize(640, 400)
            node.setInterleaved(False)
            node.setColorOrder(depthai.ColorCameraProperties.ColorOrder.RGB)
            node.preview.link(script.inputs[meta.script_input_name])

        stereo_node = pipeline.create(depthai.node.StereoDepth)
        stereo_node.setDefaultProfilePreset(depthai.node.StereoDepth.PresetMode.HIGH_ACCURACY)

        self.get_logger().info('Starting stereo enable stuff')

        left_cam_node.isp.link(script.inputs['left_stereo_script_in'])
        left_stereo_toggle_xin = pipeline.create(depthai.node.XLinkIn)
        left_stereo_toggle_xin.setStreamName('left_stereo_toggle_in')
        left_stereo_toggle_xin.out.link(script.inputs['left_stereo_toggle'])
        script.outputs['left_stereo_script_out'].link(stereo_node.left)

        right_cam_node.isp.link(script.inputs['right_stereo_script_in'])
        right_stereo_toggle_xin = pipeline.create(depthai.node.XLinkIn)
        right_stereo_toggle_xin.setStreamName('right_stereo_toggle_in')
        right_stereo_toggle_xin.out.link(script.inputs['right_stereo_toggle'])
        script.outputs['right_stereo_script_out'].link(stereo_node.right)

        # left_cam_node.isp.link(stereo_node.left)
        # right_cam_node.isp.link(stereo_node.right)

        self.get_logger().info('Finished stereo enable stuff')

        stereo_node.rectifiedLeft.link(script.inputs[self.stream_metas_dict[CAM_IDS.LUX_LEFT_RECT].script_input_name])
        stereo_node.rectifiedRight.link(script.inputs[self.stream_metas_dict[CAM_IDS.LUX_RIGHT_RECT].script_input_name])
        # stereo_node.disparity.link(script.inputs[self.stream_metas_dict[CAM_IDS.LUX_DISPARITY].script_input_name])
        # stereo_node.depth.link(script.inputs[self.stream_metas_dict[CAM_IDS.LUX_DEPTH].script_input_name])

        self.get_logger().info('Starting stream meta loop')

        for stream_meta in self.stream_metas:
            # Camera toggler -> script [script_toggle_name]
            toggle_xin = pipeline.create(depthai.node.XLinkIn)
            toggle_xin.setStreamName(stream_meta.toggle_in_stream_name)
            toggle_xin.out.link(script.inputs[stream_meta.script_toggle_name])

            # script [script_output_name] -> cam_xout
            frame_xout = pipeline.create(depthai.node.XLinkOut)
            frame_xout.setStreamName(stream_meta.out_stream_name)
            frame_xout.input.setBlocking(False)
            frame_xout.input.setQueueSize(1)
            script.outputs[stream_meta.script_output_name].link(frame_xout.input)

        self.get_logger().info('Deploying...')

        # Deploy pipeline to device
        while True:
            try:
                self.device = depthai.Device(pipeline).__enter__()
            except RuntimeError as e:
                self.get_logger().warning('Error uploading to Luxonis cam, retrying (see cam_driver to enable more details)...')
                # Uncomment to get more details about errors
                # These are usually just "the cam is disconnected", but can be other things
                # self.get_logger().warning(e)
                continue
            break

        self.get_logger().info('Getting queues')

        self.left_stereo_toggle_queue = self.device.getInputQueue('left_stereo_toggle_in')
        self.right_stereo_toggle_queue = self.device.getInputQueue('right_stereo_toggle_in')
        self.toggle_queues = {cam_id: self.device.getInputQueue(meta.toggle_in_stream_name) for cam_id, meta in self.stream_metas_dict.items()}
        self.frame_output_queues = {cam_id: self.device.getOutputQueue(meta.out_stream_name) for cam_id, meta in self.stream_metas_dict.items()}

        self.get_logger().info('Done creating pipeline')

    def spin(self) -> None:
        ##########################################################################################
        # TODO: Try sending toggles only when they need to change (i.e. in the service callback) #
        # Maybe the queues are filling up and that's hanging the node at "Spinning b2" below     #
        ##########################################################################################

        for cam_id, output_queue in self.frame_output_queues.items():
            if self.stream_metas_dict[cam_id].tx_flag:
                self.frame_publishers.try_get_publish(self.stream_metas[cam_id].topic, output_queue)

        self.get_logger().info('Spinning a')

        enable_stereo = False
        for cam_id in STREAMS_THAT_NEED_STEREO:
            if self.stream_metas_dict[cam_id].tx_flag:
                enable_stereo = True
                break

        self.get_logger().info('Spinning b')

        buf = depthai.Buffer()  # TODO: can we create this once and reuse?
        self.get_logger().info('Spinning b1')
        buf.setData(enable_stereo)
        self.get_logger().info('Spinning b2')
        self.left_stereo_toggle_queue.send(buf)
        self.get_logger().info('Spinning b3')
        self.right_stereo_toggle_queue.send(buf)

        self.get_logger().info('Spinning c')

        for cam_id, toggle_queue in self.toggle_queues.items():
            buf = depthai.Buffer()  # TODO: can we create this once and reuse?
            buf.setData(self.stream_metas_dict[cam_id].tx_flag)
            toggle_queue.send(buf)

        self.get_logger().info('Spinning d')

        # disparity_frame = self.disparity_queue.tryGet()  # blocking call, will wait until a new data has arrived

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
            rclpy.logging.get_logger('DEBUGGING').info('spinning')
            rclpy.spin_once(driver_node, executor=executor, timeout_sec=0)
            driver_node.spin()
    finally:
        driver_node.shutdown()
