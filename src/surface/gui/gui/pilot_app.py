import enum

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QScreen
from PyQt6.QtWidgets import QHBoxLayout, QVBoxLayout, QWidget

from gui.app import App
from gui.widgets.arm import Arm
from gui.widgets.flood_warning import FloodWarning
from gui.widgets.livestream_header import LivestreamHeader
from gui.widgets.manip_status import ManipStatus
from gui.widgets.timer import TimerDisplay
from gui.widgets.video_widget import (
    CameraDescription,
    CameraManager,
    CameraType,
    SwitchableVideoWidget,
    VideoWidget,
)
from rov_msgs.srv import CameraManage

CAM0_TOPIC = 'cam0/image_raw'
CAM1_TOPIC = 'cam1/image_raw'


class GuiType(enum.Enum):
    PILOT = 'pilot'
    LIVESTREAM = 'livestream'
    DEBUG = 'debug'


TWO_MONITOR = 2
THREE_MONITOR = 3

# Use 1 or 2 to launch fullscreen on the corresponding monitor
# Use `None` to launch in windowed mode
TWO_MONITOR_CONFIG: dict[GuiType, int | None] = {
    GuiType.PILOT: None,
    GuiType.LIVESTREAM: 1,
    GuiType.DEBUG: None,
}
THREE_MONITOR_CONFIG: dict[GuiType, int | None] = {
    GuiType.PILOT: 2,
    GuiType.LIVESTREAM: 1,
    GuiType.DEBUG: None,
}


class PilotApp(App):
    def __init__(self) -> None:
        super().__init__('pilot_gui_node')

        main_layout = QVBoxLayout()
        self.setLayout(main_layout)

        simulation_param = self.node.declare_parameter('simulation', value=False)
        gui_param = self.node.declare_parameter('gui', 'pilot')

        mono_cam_type = CameraType.SIMULATION if simulation_param.value else CameraType.ETHERNET

        gui_type = GuiType(gui_param.value)

        if gui_type == GuiType.PILOT:
            # TODO: Maybe remove the PILOT/DEBUG distinction now that PILOT is horizontal again
            self.setWindowTitle('Pilot GUI - CWRUbotix ROV 2025')

            video_layout = QHBoxLayout()

            for video_widget in PilotApp.make_video_widgets(mono_cam_type, 721, 541):
                video_layout.addWidget(video_widget, alignment=Qt.AlignmentFlag.AlignHCenter)

            main_layout.addLayout(video_layout)
            main_layout.addLayout(PilotApp.make_bottom_bar())

        elif gui_type == GuiType.LIVESTREAM:
            top_bar = QHBoxLayout()
            top_bar.addWidget(LivestreamHeader())
            top_bar.addWidget(TimerDisplay(), 2)
            arm_widget = Arm()
            arm_widget.setMaximumWidth(300)
            top_bar.addWidget(arm_widget, 2)

            main_layout.addLayout(top_bar)

            self.setWindowTitle('Livestream GUI - CWRUbotix ROV 2025')

            video_layout = QHBoxLayout()

            for video_widget in PilotApp.make_video_widgets(mono_cam_type, 920, 690):
                video_layout.addWidget(video_widget, alignment=Qt.AlignmentFlag.AlignHCenter)
            video_layout.setSpacing(0)

            main_layout.addLayout(video_layout)
            main_layout.addStretch()

        else:
            self.setWindowTitle('Debug GUI - CWRUbotix ROV 2025')

            video_layout = QHBoxLayout()

            for video_widget in PilotApp.make_video_widgets(mono_cam_type, 721, 541):
                video_layout.addWidget(video_widget, alignment=Qt.AlignmentFlag.AlignHCenter)

            main_layout.addLayout(video_layout)
            main_layout.addLayout(PilotApp.make_bottom_bar())

        self.apply_monitor_config(gui_type)

    @staticmethod
    def make_bottom_bar() -> QHBoxLayout:
        """Generate a bottom pane used by multiple gui types.

        Returns
        -------
        QHBoxLayout
            The layout containing the bottom bar widgets
        """
        bottom_screen_layout = QHBoxLayout()

        left_layout = QVBoxLayout()
        left_layout.addWidget(ManipStatus('left'))
        left_layout.addWidget(TimerDisplay())
        bottom_screen_layout.addLayout(left_layout)

        flood_widget = FloodWarning()
        bottom_screen_layout.addWidget(
            flood_widget, alignment=Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignBottom
        )

        right_layout = QVBoxLayout()
        right_layout.addWidget(ManipStatus('right'))
        right_layout.addWidget(Arm())
        right_container = QWidget()
        right_container.setLayout(right_layout)
        bottom_screen_layout.addWidget(
            right_container, alignment=Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignBottom
        )

        return bottom_screen_layout

    @staticmethod
    def make_video_widgets(
        mono_cam_type: CameraType, frame_width: int, frame_height: int
    ) -> tuple[VideoWidget, VideoWidget]:
        """Make the (switchable) VideoWidgets for any of the GUI types.

        Parameters
        ----------
        mono_cam_type : CameraType
            the CameraType for the monocular cams (probably ETHERNET or SIMULATION)
        frame_width : int
            width to display each frame at in px
        frame_height : int
            height to display each frame at in px

        Returns
        -------
        tuple[VideoWidget, VideoWidget]
            the left/top and right/bottom VideoWidgets
        """
        return (
            SwitchableVideoWidget(
                (
                    CameraDescription(
                        mono_cam_type,
                        CAM0_TOPIC,
                        'Forward Camera',
                        frame_width,
                        frame_height,
                        CameraManager('manage_flir', CameraManage.Request.FLIR_FRONT),
                    ),
                ),
                'switch_left_stream',
            ),
            SwitchableVideoWidget(
                (
                    CameraDescription(
                        mono_cam_type,
                        CAM1_TOPIC,
                        'Down Camera',
                        frame_width,
                        frame_height,
                        CameraManager('manage_flir', CameraManage.Request.FLIR_DOWN),
                    ),
                    CameraDescription(
                        CameraType.DEPTH,
                        'lux_raw/image_raw',
                        'Dual Left Eye',
                        frame_width,
                        frame_height,
                        CameraManager('manage_luxonis', CameraManage.Request.LUX_LEFT),
                    ),
                    CameraDescription(
                        CameraType.DEPTH,
                        'lux_raw/image_raw',
                        'Dual Right Eye',
                        frame_width,
                        frame_height,
                        CameraManager('manage_luxonis', CameraManage.Request.LUX_RIGHT),
                    ),
                ),
                'switch_right_stream',
            ),
        )

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
        if len(monitors) == TWO_MONITOR:
            monitor_id = TWO_MONITOR_CONFIG[gui_type]
        elif len(monitors) >= THREE_MONITOR:
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
