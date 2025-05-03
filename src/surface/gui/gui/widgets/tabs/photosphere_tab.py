from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QHBoxLayout, QVBoxLayout, QWidget

from gui.widgets.arm import Arm
from gui.widgets.heartbeat import HeartbeatWidget
from gui.widgets.ip_widget import IPWidget
from gui.widgets.logger import Logger
from gui.widgets.thruster_tester import ThrusterTester
from gui.widgets.video_widget import CameraDescription, CameraType, VideoWidget

FISHEYE1_TOPIC = 'fisheye1_image'
FISHEYE2_TOPIC = 'fisheye2_image'

class PhotosphereTab(QWidget):
    def __init__(self) -> None:
        super().__init__()

        top_bar = QHBoxLayout()

        fisheye1_camera_description = CameraDescription(CameraType.PHOTOSPHERE, FISHEYE1_TOPIC, "Fisheye 1")
        fisheye2_camera_description = CameraDescription(CameraType.PHOTOSPHERE, FISHEYE2_TOPIC, "Fisheye 2")

        top_bar.addWidget(
            VideoWidget(fisheye1_camera_description), alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft
        )
        top_bar.addWidget(
            VideoWidget(fisheye2_camera_description), alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft
        )

        root_layout = QVBoxLayout()
        root_layout.addLayout(top_bar)
        root_layout.addStretch()
        root_layout.addWidget(Logger())
        self.setLayout(root_layout)
