from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtCore import Qt

from gui.modules.video_widget import SwitchableVideoWidget
from gui.modules.arm import Arm
from gui.app import App


class PilotApp(App):
    def __init__(self):
        super().__init__('pilot_gui_node')

        self.setWindowTitle('Pilot GUI - CWRUbotix ROV 2024')

        layout: QHBoxLayout = QHBoxLayout()
        self.setLayout(layout)

        self.video_area = SwitchableVideoWidget(["/front_cam/image_raw",
                                                 "/bottom_cam/image_raw",
                                                 "/camera/color/image_raw"],
                                                ["Front Camera",
                                                 "Bottom Camera",
                                                 "Depth Camera"],
                                                "camera_switch")
        layout.addWidget(self.video_area, alignment=Qt.AlignCenter)

        self.arm: Arm = Arm()
        layout.addWidget(self.arm, alignment=Qt.AlignRight | Qt.AlignBottom)


def run_gui_pilot():
    PilotApp().run_gui()