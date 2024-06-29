from gui.app import App
from gui.widgets.arm import Arm
from gui.widgets.timer import TimerDisplay
from gui.widgets.flood_warning import FloodWarning
from gui.widgets.video_widget import (CameraDescription, CameraType,
                                      VideoWidget)
from gui.widgets.livestream_header import LivestreamHeader

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QHBoxLayout, QVBoxLayout
from PyQt6.QtGui import QScreen

import enum


FRONT_CAM_TOPIC = 'front_cam/image_raw'
BOTTOM_CAM_TOPIC = 'bottom_cam/image_raw'


class GuiType(enum.Enum):
    PILOT = "pilot"
    LIVESTREAM = "livestream"
    DEBUG = "debug"


TWO_MONITOR_CONFIG: dict[GuiType, int | None] = {GuiType.PILOT: None, GuiType.LIVESTREAM: 1}
THREE_MONITOR_CONFIG: dict[GuiType, int | None] = {GuiType.PILOT: 2, GuiType.LIVESTREAM: 1}


class PilotApp(App):
    def __init__(self) -> None:
        super().__init__('pilot_gui_node')

        main_layout = QVBoxLayout()
        self.setLayout(main_layout)

        simulation_param = self.node.declare_parameter('simulation', False)
        gui_param = self.node.declare_parameter('gui', 'pilot')

        if simulation_param.value:
            front_cam_type = CameraType.SIMULATION
            bottom_cam_type = CameraType.SIMULATION
        else:
            front_cam_type = CameraType.ETHERNET
            bottom_cam_type = CameraType.ETHERNET

        gui_type = GuiType(gui_param.value)

        if gui_type == GuiType.PILOT:
            self.setWindowTitle('Pilot GUI - CWRUbotix ROV 2024')

            front_cam_description = CameraDescription(
                front_cam_type,
                FRONT_CAM_TOPIC,
                "Front Camera",
                1280, 720
            )
            bottom_cam_description = CameraDescription(
                bottom_cam_type,
                BOTTOM_CAM_TOPIC,
                "Bottom Camera",
                1280, 720
            )

            main_layout.addWidget(VideoWidget(front_cam_description),
                                  alignment=Qt.AlignmentFlag.AlignHCenter)
            main_layout.addWidget(
                VideoWidget(bottom_cam_description),
                alignment=Qt.AlignmentFlag.AlignHCenter
            )

            main_layout.addLayout(self.make_bottom_bar())

        elif gui_type == GuiType.LIVESTREAM:
            top_bar = QHBoxLayout()
            top_bar.addWidget(LivestreamHeader())
            top_bar.addWidget(TimerDisplay(), 2)
            arm_widget = Arm()
            arm_widget.setMaximumWidth(300)
            top_bar.addWidget(arm_widget, 2)

            main_layout.addLayout(top_bar)

            self.setWindowTitle('Livestream GUI - CWRUbotix ROV 2024')

            front_cam_description = CameraDescription(
                front_cam_type,
                FRONT_CAM_TOPIC,
                "Forward Camera",
                920, 690
            )
            bottom_cam_description = CameraDescription(
                bottom_cam_type,
                BOTTOM_CAM_TOPIC,
                "Down Camera",
                920, 690
            )

            video_layout = QHBoxLayout()

            video_layout.addWidget(VideoWidget(front_cam_description),
                                   alignment=Qt.AlignmentFlag.AlignHCenter)
            video_layout.addWidget(VideoWidget(bottom_cam_description),
                                   alignment=Qt.AlignmentFlag.AlignHCenter)
            video_layout.setSpacing(0)

            main_layout.addLayout(video_layout)
            main_layout.addStretch()

        else:
            self.setWindowTitle('Debug GUI - CWRUbotix ROV 2024')

            front_cam_description = CameraDescription(
                front_cam_type,
                FRONT_CAM_TOPIC,
                "Front Camera",
                721, 541
            )
            bottom_cam_description = CameraDescription(
                bottom_cam_type,
                BOTTOM_CAM_TOPIC,
                "Bottom Camera",
                721, 541
            )

            video_layout = QHBoxLayout()

            video_layout.addWidget(VideoWidget(front_cam_description),
                                   alignment=Qt.AlignmentFlag.AlignHCenter)
            video_layout.addWidget(
                VideoWidget(bottom_cam_description),
                alignment=Qt.AlignmentFlag.AlignHCenter
            )

            main_layout.addLayout(video_layout)
            main_layout.addLayout(self.make_bottom_bar())

        self.apply_monitor_config(gui_type)

    def make_bottom_bar(self) -> QHBoxLayout:
        """Generate a bottom pane used by multiple gui types.

        Returns
        -------
        QHBoxLayout
            The layout containing the bottom bar widgets
        """
        bottom_screen_layout = QHBoxLayout()

        timer = TimerDisplay()
        bottom_screen_layout.addWidget(timer)

        flood_widget = FloodWarning()
        bottom_screen_layout.addWidget(flood_widget, alignment=Qt.AlignmentFlag.AlignHCenter |
                                       Qt.AlignmentFlag.AlignBottom)

        arm = Arm()
        bottom_screen_layout.addWidget(arm, alignment=Qt.AlignmentFlag.AlignRight |
                                       Qt.AlignmentFlag.AlignBottom)

        return bottom_screen_layout

    def apply_monitor_config(self, gui_type: GuiType) -> None:
        """Fullscreen the app to a specific monitor, depending on gui_type and the monitor config.

        Either fullscreens the app to a monitor specified by TWO_MONITOR_CONFIG or
        THREE_MONITOR_CONFIG (depending on the number of monitors present), or does nothing if no
        config exists for the number of monitors and gui type

        Parameters
        ----------
        gui_type : GuiType
            The type of gui that is being initialized
        """
        screen = self.screen()
        if screen is None:
            return

        monitors = QScreen.virtualSiblings(screen)

        monitor_id: int | None
        if len(monitors) == 2:
            monitor_id = TWO_MONITOR_CONFIG[gui_type]
        elif len(monitors) >= 3:
            monitor_id = THREE_MONITOR_CONFIG[gui_type]
        else:
            return

        if monitor_id is None:
            return

        monitor = monitors[monitor_id].availableGeometry()
        self.move(monitor.left(), monitor.top())
        self.showFullScreen()


def run_gui_pilot() -> None:
    PilotApp().run_gui()
