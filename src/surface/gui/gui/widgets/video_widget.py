from dataclasses import dataclass
from enum import IntEnum
from typing import Sequence, NamedTuple

import cv2
import numpy as np
from cv_bridge import CvBridge
from numpy.typing import NDArray
from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import QLabel, QPushButton, QVBoxLayout, QWidget
from rclpy.qos import qos_profile_default
from sensor_msgs.msg import Image

from gui.gui_node import GUINode
from rov_msgs.msg import VideoWidgetSwitch
from rov_msgs.srv import CameraManage

# TODO: Ubuntu26+
# Our own implementation of cv2.typing.MatLike until cv2.typing exists in a future ubuntu release
# This what is actually implemented in cv2.typing:
# MatLike = cv2.mat_wrapper.Mat | NDArray[np.integer[Any] | np.floating[Any]]
# This should be possible in a newer version of mypy:
# MatLike = NDArray[np.integer[Any] | np.floating[Any]]
MatLike = NDArray[np.generic]

WIDTH = 721
HEIGHT = 541
# 1 Pixel larger than actual pixel dimensions


COLOR = 3
GREY_SCALE = 2


class CameraType(IntEnum):
    """
    Enum Class for defining Camera Types.

    Currently only Ethernet changes behavior.
    """

    USB = 1
    ETHERNET = 2
    DEPTH = 3
    SIMULATION = 4

@dataclass
class CameraManager:
    def __init__(self, topic_name: str, camera_id: int) -> None:
        self.camera_id = camera_id
        self.topic_name = topic_name
        self.client = GUINode().create_client_multithreaded(CameraManage, topic_name)

    def set_cam_state(self, *, on: bool) -> None:
        GUINode().send_request_multithreaded(self.client, CameraManage.Request(cam=self.camera_id, on=on))

class CameraDescription(NamedTuple):
    """
    Generic CameraDescription describes each camera for a VideoWidget.

    Parameters
    ----------
    type: CameraType
        Describes the type of Camera.
    topic: str
        The topic to listen on, by default cam
    label: str
        The label of the camera, by default Camera
    width: int
        The width of the Camera Stream, by default WIDTH constant.
    height: int
        The height of the Camera Stream, by default HEIGHT constant.

    """

    type: CameraType
    topic: str = 'cam'
    label: str = 'Camera'
    width: int = WIDTH
    height: int = HEIGHT
    manager: CameraManager | None = None


class VideoWidget(QWidget):
    """A single video stream widget."""

    update_big_video_signal = pyqtSignal(QWidget)
    handle_frame_signal = pyqtSignal(Image)

    def __init__(self, camera_description: CameraDescription) -> None:
        super().__init__()

        self.camera_description = camera_description

        layout = QVBoxLayout()
        self.setLayout(layout)

        self.video_frame_label = QLabel()
        self.video_frame_label.setText(f'This topic had no frame: {camera_description.topic}')
        layout.addWidget(self.video_frame_label)

        self.label = QLabel(camera_description.label)
        self.label.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self.label.setStyleSheet('QLabel { font-size: 35px; }')
        layout.addWidget(self.label, Qt.AlignmentFlag.AlignHCenter)

        self.cv_bridge = CvBridge()

        self.handle_frame_signal.connect(self.handle_frame)
        self.camera_subscriber = GUINode().create_signal_subscription(
            Image, camera_description.topic, self.handle_frame_signal
        )

    @pyqtSlot(Image)
    def handle_frame(self, frame: Image) -> None:
        cv_image = self.cv_bridge.imgmsg_to_cv2(frame, desired_encoding='passthrough')

        qt_image: QImage = self.convert_cv_qt(
            cv_image, self.camera_description.width, self.camera_description.height
        )

        self.video_frame_label.setPixmap(QPixmap.fromImage(qt_image))

    def convert_cv_qt(self, cv_img: MatLike, width: int = 0, height: int = 0) -> QImage:
        """Convert from an opencv image to QPixmap."""
        if self.camera_description.type == CameraType.ETHERNET:
            # Switches ethernet's color profile from BayerBGR to BGR
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BAYER_BGGR2BGR)

        # Color image
        if len(cv_img.shape) == COLOR:
            h, w, ch = cv_img.shape
            bytes_per_line = ch * w

            img_format = QImage.Format.Format_RGB888

        # Grayscale image
        elif len(cv_img.shape) == GREY_SCALE:
            h, w = cv_img.shape
            bytes_per_line = w

            img_format = QImage.Format.Format_Grayscale8

        else:
            raise ValueError('Somehow not color or grayscale image.')

        qt_image = QImage(cv_img.data, w, h, bytes_per_line, img_format)
        qt_image = qt_image.scaled(width, height, Qt.AspectRatioMode.KeepAspectRatio)

        return qt_image


class SwitchableVideoWidget(VideoWidget):
    BUTTON_WIDTH = 150

    controller_signal = pyqtSignal(VideoWidgetSwitch)

    def __init__(
        self,
        camera_descriptions: Sequence[CameraDescription],
        controller_button_topic: str,
        default_cam_num: int = 0,
    ) -> None:
        self.camera_descriptions = camera_descriptions
        self.active_cam = default_cam_num

        self.num_of_cams = len(camera_descriptions)

        super().__init__(camera_descriptions[self.active_cam])

        self.button: QPushButton = QPushButton(camera_descriptions[self.active_cam].label)
        self.button.setMaximumWidth(self.BUTTON_WIDTH)
        self.button.clicked.connect(self.gui_camera_switch)

        layout = self.layout()
        if isinstance(layout, QVBoxLayout):
            layout.addWidget(self.button, alignment=Qt.AlignmentFlag.AlignCenter)
        else:
            GUINode().get_logger().error('Missing Layout')

        self.controller_signal.connect(self.controller_camera_switch)
        self.controller_publisher = GUINode().create_publisher(
            VideoWidgetSwitch, controller_button_topic, qos_profile_default
        )
        self.controller_subscriber = GUINode().create_signal_subscription(
            VideoWidgetSwitch, controller_button_topic, self.controller_signal
        )

    @pyqtSlot(VideoWidgetSwitch)
    def controller_camera_switch(self, switch: VideoWidgetSwitch) -> None:
        self.camera_switch(index=switch.index, relative=switch.relative)

    def gui_camera_switch(self) -> None:
        self.controller_publisher.publish(VideoWidgetSwitch(relative=True, index=1))

    def camera_switch(self, index: int, *, relative: bool) -> None:
        if relative:
            self.active_cam += index
        else:
            self.active_cam = index
        self.active_cam %= self.num_of_cams

        # Update Camera Description
        new_cam_description = self.camera_descriptions[self.active_cam]

        if new_cam_description.topic != self.camera_description.topic:
            GUINode().destroy_subscription(self.camera_subscriber)
            self.camera_subscriber = GUINode().create_signal_subscription(
                Image, new_cam_description.topic, self.handle_frame_signal
            )
        if self.camera_description.manager is not None:
            self.camera_description.manager.set_cam_state(on=False)
        if new_cam_description.manager is not None:
            new_cam_description.manager.set_cam_state(on=True)

        self.button.setText(new_cam_description.label)

        # Updates text for info when no frame received.
        self.video_frame_label.setText(f'This topic had no frame: {new_cam_description.topic}')
        self.label.setText(new_cam_description.label)

        self.camera_description = new_cam_description


class PauseableVideoWidget(VideoWidget):
    """A single video stream widget that can be paused and played."""

    BUTTON_WIDTH = 150
    PAUSED_TEXT = 'Play'
    PLAYING_TEXT = 'Pause'

    def __init__(self, camera_description: CameraDescription) -> None:
        super().__init__(camera_description)

        self.button = QPushButton(self.PLAYING_TEXT)
        self.button.setMaximumWidth(self.BUTTON_WIDTH)
        self.button.clicked.connect(self.toggle)

        layout = self.layout()
        if isinstance(layout, QVBoxLayout):
            layout.addWidget(self.button, alignment=Qt.AlignmentFlag.AlignCenter)
        else:
            GUINode().get_logger().error('Missing Layout')

        self.is_paused = False

    @pyqtSlot(Image)
    def handle_frame(self, frame: Image) -> None:
        if not self.is_paused:
            super().handle_frame(frame)

    def toggle(self) -> None:
        """Toggle whether this widget is paused or playing."""
        self.is_paused = not self.is_paused
        self.button.setText(self.PAUSED_TEXT if self.is_paused else self.PLAYING_TEXT)
