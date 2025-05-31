from PyQt6.QtWidgets import QHBoxLayout, QPushButton, QVBoxLayout, QWidget

from gui.widgets.video_widget import CameraDescription, CameraManager, CameraType, VideoWidget, SwitchableVideoWidget
from rov_msgs.srv import CameraManage


FRAME_WIDTH = 921
FRAME_HEIGHT = 690

class ShipwreckTab(QWidget):
    def __init__(self) -> None:
        super().__init__()

        cam_layout = QHBoxLayout()

        left_eye = SwitchableVideoWidget(
            (
                CameraDescription(CameraType.DEPTH, 'rect_left/image_raw', 'Stream stopped',
                                  FRAME_WIDTH, FRAME_HEIGHT),
                CameraDescription(
                    CameraType.DEPTH,
                    'rect_left/image_raw',
                    'Dual Left Eye',
                    FRAME_WIDTH,
                    FRAME_HEIGHT,
                    CameraManager('manage_luxonis', CameraManage.Request.LUX_LEFT_RECT),
                ),
            ),
            'switch_rect_left_stream'
        )

        right_eye = SwitchableVideoWidget(
            (
                CameraDescription(CameraType.DEPTH, 'rect_right/image_raw', 'Stream stopped',
                                  FRAME_WIDTH, FRAME_HEIGHT),
                CameraDescription(
                    CameraType.DEPTH,
                    'rect_right/image_raw',
                    'Dual Right Eye',
                    FRAME_WIDTH,
                    FRAME_HEIGHT,
                    CameraManager('manage_luxonis', CameraManage.Request.LUX_RIGHT_RECT),
                ),
            ),
            'switch_rect_left_stream'
        )

        cam_layout.addWidget(left_eye)
        cam_layout.addWidget(right_eye)

        button_layout = QHBoxLayout()
        on_button = QPushButton('Start stream')
        off_button = QPushButton('Stop stream')
        button_layout.addWidget(on_button)
        button_layout.addWidget(off_button)

        root_layout = QVBoxLayout()
        root_layout.addLayout(cam_layout)
        root_layout.addLayout(button_layout)

        self.setLayout(root_layout)
