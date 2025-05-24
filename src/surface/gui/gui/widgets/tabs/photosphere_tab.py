from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot

from PyQt6.QtWidgets import QHBoxLayout, QVBoxLayout, QWidget, QPushButton, QLabel

from gui.widgets.logger import Logger
from gui.widgets.video_widget import CameraDescription, CameraType, VideoWidget
from gui.gui_node import GUINode
from rclpy.qos import qos_profile_default
from rov_msgs.srv import GeneratePhotosphere


FISHEYE1_TOPIC = '/surface/fisheye1_image'
FISHEYE2_TOPIC = '/surface/fisheye2_image'

class PhotosphereTab(QWidget):


    photosphere_response_signal = pyqtSignal(GeneratePhotosphere.Response)

    def __init__(self) -> None:
        super().__init__()

        top_bar = QHBoxLayout()

        # fisheye1_camera_description = CameraDescription(CameraType.PHOTOSPHERE, FISHEYE1_TOPIC, "Fisheye 1")
        # fisheye2_camera_description = CameraDescription(CameraType.PHOTOSPHERE, FISHEYE2_TOPIC, "Fisheye 2")
        
        
        self.photosphere_client = GUINode().create_client_multithreaded(GeneratePhotosphere, 'generate_photosphere')

        self.photosphere_response_signal.connect(self.photosphere_status)

        # top_bar.addWidget(
        #     VideoWidget(fisheye1_camera_description), alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft
        # )
        # top_bar.addWidget(
        #     VideoWidget(fisheye2_camera_description), alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft
        # )

        root_layout = QVBoxLayout()
        root_layout.addLayout(top_bar)

        self.photosphere_button = QPushButton('Generate Photosphere')
        self.photosphere_button.setMinimumHeight(60)
        self.photosphere_button.setMinimumWidth(120)
        self.photosphere_button.clicked.connect(self.generate_clicked)

        root_layout.addWidget(self.photosphere_button)

        self.photosphere_status_label = QLabel('No Photosphere Generated')

        root_layout.addWidget(self.photosphere_status_label)

        root_layout.addStretch()
        root_layout.addWidget(Logger())
        self.setLayout(root_layout)

    def generate_clicked(self) -> None:
        GUINode().send_request_multithreaded(self.photosphere_client, GeneratePhotosphere.Request(), self.photosphere_response_signal)

    @pyqtSlot(GeneratePhotosphere.Response)
    def photosphere_status(self, res: GeneratePhotosphere.Response) -> None:
        if not res or not res.generated:
            self.photosphere_status_label.setText('Failed to generate photosphere')
        else:
            self.photosphere_status_label.setText('Photosphere generated')
    
