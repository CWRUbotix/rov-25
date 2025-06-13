from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot, QUrl

from PyQt6.QtWidgets import QHBoxLayout, QVBoxLayout, QWidget, QPushButton, QLabel
import PyQt6.QtWebEngineWidgets
from PyQt6.QtWebEngineWidgets import QWebEngineView



from gui.widgets.logger import Logger
from gui.widgets.video_widget import CameraDescription, CameraType, VideoWidget
from gui.gui_node import GUINode
from rclpy.qos import qos_profile_default
from rov_msgs.srv import GeneratePhotosphere

class PhotosphereDisplayTab(QWidget):


    # photosphere_response_signal = pyqtSignal(GeneratePhotosphere.Response)

    def __init__(self) -> None:
        super().__init__()

        root_layout = QVBoxLayout()


        view = QWebEngineView()
        view.load(QUrl('http://localhost:8080/'))

        root_layout.addWidget(view)

        # self.photosphere_button = QPushButton('Generate Photosphere')
        # self.photosphere_button.setMinimumHeight(60)
        # self.photosphere_button.setMinimumWidth(120)
        # self.photosphere_button.clicked.connect(self.generate_clicked)

        # root_layout.addWidget(self.photosphere_button)

        # self.photosphere_status_label = QLabel('No Photosphere Generated')

        # root_layout.addWidget(self.photosphere_status_label)

        root_layout.addStretch()
        root_layout.addWidget(Logger())
        self.setLayout(root_layout)

    # def generate_clicked(self) -> None:
    #     GUINode().send_request_multithreaded(self.photosphere_client, GeneratePhotosphere.Request(), self.photosphere_response_signal)
    #     self.photosphere_status_label.setText('Generating Photosphere')

    # @pyqtSlot(GeneratePhotosphere.Response)
    # def photosphere_status(self, res: GeneratePhotosphere.Response) -> None:
    #     if not res or not res.generated:
    #         self.photosphere_status_label.setText('Failed to generate photosphere')
    #     else:
    #         self.photosphere_status_label.setText('Successfully Generated Photosphere')