from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import QGridLayout, QHBoxLayout, QLabel, QVBoxLayout, QWidget

from gui.gui_node import GUINode
from gui.styles.custom_styles import ButtonIndicator, WidgetState
from gui.widgets.logger import Logger
from gui.widgets.video_widget import CameraDescription, CameraType, VideoWidget
from rov_msgs.srv import GeneratePhotosphere, TakePhotosphere

FISHEYE1_TOPIC = 'photosphere/image_1'
FISHEYE2_TOPIC = 'photosphere/image_2'


class PhotosphereTab(QWidget):
    take_photos_response_signal = pyqtSignal(TakePhotosphere.Response)
    generate_response_signal = pyqtSignal(GeneratePhotosphere.Response)

    def __init__(self) -> None:
        super().__init__()

        video_pane = QHBoxLayout()

        fisheye1_camera_description = CameraDescription(
            CameraType.PHOTOSPHERE, FISHEYE1_TOPIC, 'Fisheye 1'
        )
        fisheye2_camera_description = CameraDescription(
            CameraType.PHOTOSPHERE, FISHEYE2_TOPIC, 'Fisheye 2'
        )

        self.take_photos_client = GUINode().create_client_multithreaded(
            TakePhotosphere, 'photosphere/take_photos'
        )
        self.generate_photosphere_client = GUINode().create_client_multithreaded(
            GeneratePhotosphere, 'photosphere/generate_photosphere'
        )

        self.generate_response_signal.connect(self.generate_response_handler)
        self.take_photos_response_signal.connect(self.take_photos_response_handler)

        video_pane.addWidget(
            VideoWidget(fisheye1_camera_description),
            alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft,
        )
        video_pane.addWidget(
            VideoWidget(fisheye2_camera_description),
            alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft,
        )

        button_pane = QGridLayout()

        self.take_photos_button = ButtonIndicator('Take Photos')
        self.take_photos_button.clicked.connect(
            lambda: self.take_photo_clicked(TakePhotosphere.Request.BOTH)
        )

        self.take_cam0_button = ButtonIndicator('Take 0')
        self.take_cam1_button = ButtonIndicator('Take 1')
        self.take_cam0_button.clicked.connect(
            lambda: self.take_photo_clicked(cam=TakePhotosphere.Request.CAM0)
        )
        self.take_cam1_button.clicked.connect(
            lambda: self.take_photo_clicked(cam=TakePhotosphere.Request.CAM1)
        )

        self.photo_buttons: dict[int, tuple[ButtonIndicator, ...]] = {
            TakePhotosphere.Request.CAM0: (self.take_cam0_button,),
            TakePhotosphere.Request.CAM1: (self.take_cam1_button,),
            TakePhotosphere.Request.BOTH: (self.take_cam0_button, self.take_cam1_button),
        }

        self.generate_button = ButtonIndicator('Generate Photosphere')
        self.generate_button.set_state(WidgetState.INACTIVE)
        self.generate_button.clicked.connect(self.generate_clicked)

        button_pane.addWidget(self.take_photos_button, 0, 0, 1, 2)
        button_pane.addWidget(self.take_cam0_button, 1, 0)
        button_pane.addWidget(self.take_cam1_button, 1, 1)
        button_pane.addWidget(self.generate_button, 2, 0, 1, 2)

        root_layout = QVBoxLayout()
        root_layout.addLayout(video_pane)
        root_layout.addLayout(button_pane)

        root_layout.addWidget(self.generate_button)

        self.photosphere_status_label = QLabel('No Photosphere Generated')

        root_layout.addWidget(self.photosphere_status_label)

        root_layout.addStretch()
        root_layout.addWidget(Logger())
        self.setLayout(root_layout)

    def take_photo_clicked(self, cam: int) -> None:
        for button in self.photo_buttons[cam]:
            button.set_state(WidgetState.INACTIVE)
        GUINode().send_request_multithreaded(
            self.take_photos_client,
            TakePhotosphere.Request(cam=cam),
            self.take_photos_response_signal,
        )

    def generate_clicked(self) -> None:
        GUINode().send_request_multithreaded(
            self.generate_photosphere_client,
            GeneratePhotosphere.Request(),
            self.generate_response_signal,
        )

    @pyqtSlot(GeneratePhotosphere.Response)
    def generate_response_handler(self, res: GeneratePhotosphere.Response) -> None:
        if not res or not res.generated:
            self.photosphere_status_label.setText('Failed to generate photosphere')
            self.generate_button.set_state(WidgetState.OFF)
        else:
            self.photosphere_status_label.setText('Photosphere generated')
            self.generate_button.set_state(WidgetState.ON)

    @pyqtSlot(TakePhotosphere.Response)
    def take_photos_response_handler(self, res: TakePhotosphere.Response) -> None:
        if not res:
            for button in self.photo_buttons[TakePhotosphere.Request.BOTH]:
                button.set_state(WidgetState.OFF)
        elif res.success:
            for button in self.photo_buttons[res.cam]:
                button.set_state(WidgetState.ON)
            if self.generate_button.current_state == WidgetState.INACTIVE and all(
                button.current_state == WidgetState.ON
                for button in self.photo_buttons[TakePhotosphere.Request.BOTH]
            ):
                self.generate_button.set_state(WidgetState.NONE)
        else:
            for button in self.photo_buttons[res.cam]:
                button.set_state(WidgetState.OFF)
