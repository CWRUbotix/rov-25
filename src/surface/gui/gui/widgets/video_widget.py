from typing import NamedTuple, Optional

import cv2
from cv2.typing import MatLike
from cv_bridge import CvBridge
from gui.gui_nodes.event_nodes.subscriber import GUIEventSubscriber
from gui.gui_nodes.event_nodes.publisher import GUIEventPublisher
from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import QLabel, QPushButton, QVBoxLayout, QWidget
from sensor_msgs.msg import Image
from enum import IntEnum

from rov_msgs.msg import CameraControllerSwitch

WIDTH = 721
HEIGHT = 541
# 1 Pixel larger than actual pixel dimensions


class CameraType(IntEnum):
    """
    Enum Class for defining Camera Types.

    Currently only Ethernet changes behavior.
    """

    USB = 1
    ETHERNET = 2
    DEPTH = 3
    SIMULATION = 4


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

        self.cv_bridge: CvBridge = CvBridge()

        self.handle_frame_signal.connect(self.handle_frame)
        self.camera_subscriber: GUIEventSubscriber = GUIEventSubscriber(
            Image, camera_description.topic, self.handle_frame_signal)

    @pyqtSlot(Image)
    def handle_frame(self, frame: Image) -> None:
        cv_image: MatLike = self.cv_bridge.imgmsg_to_cv2(
            frame, desired_encoding='passthrough')

        qt_image: QImage = self.convert_cv_qt(cv_image,
                                              self.camera_description.width,
                                              self.camera_description.height)

        self.video_frame_label.setPixmap(QPixmap.fromImage(qt_image))

    def convert_cv_qt(self, cv_img: MatLike, width: int = 0, height: int = 0) -> QImage:
        """Convert from an opencv image to QPixmap."""
        if self.camera_description.type == CameraType.ETHERNET:
            # Switches ethernet's color profile from BayerBGR to BGR
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BAYER_BGGR2BGR)

        # Color image
        if len(cv_img.shape) == 3:
            h, w, ch = cv_img.shape
            bytes_per_line = ch * w

            img_format = QImage.Format.Format_RGB888

        # Grayscale image
        elif len(cv_img.shape) == 2:
            h, w = cv_img.shape
            bytes_per_line = w

            img_format = QImage.Format.Format_Grayscale8

        else:
            raise ValueError("Somehow not color or grayscale image.")

        qt_image = QImage(cv_img.data, w, h, bytes_per_line, img_format)
        qt_image = qt_image.scaled(width, height, Qt.AspectRatioMode.KeepAspectRatio)

        return qt_image


class SwitchableVideoWidget(VideoWidget):
    """A single video stream widget that can be paused and played."""

    BUTTON_WIDTH = 150

    controller_signal = pyqtSignal(CameraControllerSwitch)

    def __init__(self, camera_descriptions: list[CameraDescription],
                 controller_button_topic: Optional[str] = None,
                 default_cam_num: int = 0):
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
            self.camera_subscriber.get_logger().error("Missing Layout")

        if controller_button_topic is not None:
            self.controller_signal.connect(self.controller_camera_switch)
            self.controller_publisher = GUIEventPublisher(CameraControllerSwitch,
                                                          controller_button_topic)
            self.controller_subscriber = GUIEventSubscriber(CameraControllerSwitch,
                                                            controller_button_topic,
                                                            self.controller_signal)

    @pyqtSlot(CameraControllerSwitch)
    def controller_camera_switch(self, switch: CameraControllerSwitch) -> None:
        self.camera_switch(switch.toggle_right)

    def gui_camera_switch(self) -> None:
        self.controller_publisher.publish(CameraControllerSwitch(toggle_right=True))

    def camera_switch(self, toggle_right: bool) -> None:
        if toggle_right:
            self.active_cam = (self.active_cam + 1) % self.num_of_cams
        else:
            self.active_cam = (self.active_cam - 1) % self.num_of_cams

        # Update Camera Description
        self.camera_description = self.camera_descriptions[self.active_cam]

        self.camera_subscriber.destroy_node()
        self.camera_subscriber = GUIEventSubscriber(
            Image, self.camera_description.topic, self.handle_frame_signal)
        self.button.setText(self.camera_description.label)

        # Updates text for info when no frame received.
        self.video_frame_label.setText(f'This topic had no frame: {self.camera_description.topic}')
        self.label.setText(self.camera_description.label)


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
            self.camera_subscriber.get_logger().error("Missing Layout")

        self.is_paused = False

    @pyqtSlot(Image)
    def handle_frame(self, frame: Image) -> None:
        if not self.is_paused:
            super().handle_frame(frame)

    def toggle(self) -> None:
        """Toggle whether this widget is paused or playing."""
        self.is_paused = not self.is_paused
        self.button.setText(self.PAUSED_TEXT if self.is_paused else self.PLAYING_TEXT)
