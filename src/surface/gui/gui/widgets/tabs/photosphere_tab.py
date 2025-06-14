from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot

from PyQt6.QtWidgets import QHBoxLayout, QVBoxLayout, QWidget, QPushButton, QLabel

from gui.widgets.logger import Logger
from gui.widgets.video_widget import CameraDescription, CameraType, VideoWidget
from gui.gui_node import GUINode
from gui.styles.custom_styles import ButtonIndicator, WidgetState
from rov_msgs.srv import GeneratePhotosphere
from std_srvs.srv import Trigger


FISHEYE1_TOPIC = 'photosphere/image_1'
FISHEYE2_TOPIC = 'photosphere/image_2'

class PhotosphereTab(QWidget):

    take_photos_response_signal = pyqtSignal(Trigger.Response)
    generate_response_signal = pyqtSignal(GeneratePhotosphere.Response)

    def __init__(self) -> None:
        super().__init__()

        video_pane = QHBoxLayout()

        fisheye1_camera_description = CameraDescription(CameraType.PHOTOSPHERE, FISHEYE1_TOPIC, "Fisheye 1")
        fisheye2_camera_description = CameraDescription(CameraType.PHOTOSPHERE, FISHEYE2_TOPIC, "Fisheye 2")
        
        
        self.take_photos_client = GUINode().create_client_multithreaded(Trigger, 'photosphere/take_photos')
        self.generate_photosphere_client = GUINode().create_client_multithreaded(GeneratePhotosphere, 'photosphere/generate_photosphere')

        self.generate_response_signal.connect(self.generate_response_handler)
        self.take_photos_response_signal.connect(self.take_photos_response_handler)

        video_pane.addWidget(
            VideoWidget(fisheye1_camera_description), alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft
        )
        video_pane.addWidget(
            VideoWidget(fisheye2_camera_description), alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft
        )

        button_pane = QHBoxLayout()

        self.take_photos_button = ButtonIndicator("Take Photos")
        self.take_photos_button.setMinimumHeight(60)
        self.take_photos_button.setMinimumWidth(150)
        self.take_photos_button.clicked.connect(self.take_photos_clicked)

        self.generate_button = ButtonIndicator('Generate Photosphere')
        self.generate_button.setMinimumHeight(60)
        self.generate_button.setMinimumWidth(150)
        self.generate_button.set_state(WidgetState.INACTIVE)
        self.generate_button.clicked.connect(self.generate_clicked)

        button_pane.addWidget(self.take_photos_button)
        button_pane.addWidget(self.generate_button)
        button_pane.addStretch()

        root_layout = QVBoxLayout()
        root_layout.addLayout(video_pane)
        root_layout.addLayout(button_pane)


        root_layout.addWidget(self.generate_button)

        self.photosphere_status_label = QLabel('No Photosphere Generated')

        root_layout.addWidget(self.photosphere_status_label)

        root_layout.addStretch()
        root_layout.addWidget(Logger())
        self.setLayout(root_layout)

    def take_photos_clicked(self) -> None:
        self.take_photos_button.set_state(WidgetState.INACTIVE)
        GUINode().send_request_multithreaded(self.take_photos_client, Trigger.Request(), self.take_photos_response_signal)

    def generate_clicked(self) -> None:
        GUINode().send_request_multithreaded(self.generate_photosphere_client, GeneratePhotosphere.Request(), self.generate_response_signal)

    @pyqtSlot(GeneratePhotosphere.Response)
    def generate_response_handler(self, res: GeneratePhotosphere.Response) -> None:
        if not res or not res.generated:
            self.photosphere_status_label.setText('Failed to generate photosphere')
            self.generate_button.set_state(WidgetState.OFF)
        else:
            self.photosphere_status_label.setText('Photosphere generated')
            self.generate_button.set_state(WidgetState.ON)

    @pyqtSlot(Trigger.Response)
    def take_photos_response_handler(self, res: Trigger.Response) -> None:
        if res and res.success:
            self.take_photos_button.set_state(WidgetState.ON)
            if self.generate_button.current_state == WidgetState.INACTIVE:
                self.generate_button.set_state(WidgetState.NONE)
        else:
            self.take_photos_button.set_state(WidgetState.OFF)
    
