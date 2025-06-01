from collections.abc import Callable
from dataclasses import dataclass
from typing import override

from PyQt6.QtCore import QEvent, QObject, Qt, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QKeyEvent, QMouseEvent
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from gui.gui_node import GUINode
from gui.widgets.video_widget import (
    CameraDescription,
    CameraManager,
    CameraType,
    ClickableLabel,
    SwitchableVideoWidget,
)
from rov_msgs.srv import CameraManage

FRAME_WIDTH = 921
FRAME_HEIGHT = 690

POINTS_PER_EYE = 2

@dataclass
class Point:
    x: int
    y: int

    @override
    def __str__(self) -> str:
        return f'({self.x}, {self.y})'

@dataclass
class KeyPoints:
    left_eye: list[Point | None]
    right_eye: list[Point | None]

    def has_all_points(self) -> bool:
        return all(point is not None for point in self.left_eye + self.right_eye) and \
                len(self.left_eye) == POINTS_PER_EYE and len(self.right_eye) == POINTS_PER_EYE

    @override
    def __str__(self) -> str:
        return f'{self.left_eye[0]}-{self.left_eye[1]} \t {self.right_eye[0]}-{self.right_eye[1]}'

class ShipwreckTab(QWidget):
    click_left_signal = pyqtSignal(QMouseEvent)
    click_right_signal = pyqtSignal(QMouseEvent)

    def __init__(self) -> None:
        super().__init__()

        self.click_left_signal.connect(self.click_left)
        self.click_right_signal.connect(self.click_right)

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
            'switch_rect_left_stream',
            make_label=lambda: ClickableLabel(self.click_left_signal)
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
            'switch_rect_left_stream',
            make_label=lambda: ClickableLabel(self.click_right_signal)
        )

        cam_layout.addWidget(left_eye)
        cam_layout.addWidget(right_eye)

        button_layout = QHBoxLayout()
        on_button = QPushButton('Start stream')
        off_button = QPushButton('Stop stream')
        self.points_label = QLabel('No points')
        button_layout.addWidget(on_button)
        button_layout.addWidget(off_button)
        button_layout.addWidget(self.points_label)

        root_layout = QVBoxLayout()
        root_layout.addLayout(cam_layout)
        root_layout.addLayout(button_layout)

        self.setLayout(root_layout)

        self.points = KeyPoints([None, None], [None, None])

        self.keys: dict[int, bool] = {
            Qt.Key.Key_1.value: False,
            Qt.Key.Key_2.value: False,
        }

    @pyqtSlot(QMouseEvent)
    def click_left(self, event: QMouseEvent) -> None:
        self.click_eye(event, is_right_eye=False)

    @pyqtSlot(QMouseEvent)
    def click_right(self, event: QMouseEvent) -> None:
        self.click_eye(event, is_right_eye=True)

    def click_eye(self, event: QMouseEvent, *, is_right_eye: bool) -> None:
        if self.keys[Qt.Key.Key_2]:
            point_idx = 1
        elif self.keys[Qt.Key.Key_1]:
            point_idx = 0
        else:
            return  # No key pressed, don't register point

        point = Point(event.pos().x(), event.pos().y())

        if is_right_eye:
            self.points.right_eye[point_idx] = point
        else:
            self.points.left_eye[point_idx] = point

        GUINode().get_logger().info(str(self.points))

        self.points_label.setText(str(self.points))

    def keyPressEvent(self, event: QKeyEvent | None) -> None:  # noqa: N802
        self.handle_key_event(event, target_value=True)

    def keyReleaseEvent(self, event: QKeyEvent | None) -> None:  # noqa: N802
        self.handle_key_event(event, target_value=False)

    def handle_key_event(self, event: QKeyEvent | None, *, target_value: bool) -> None:
        # Shouldn't ever happen, just type narrowing
        if event is None:
            return

        if not event.isAutoRepeat() and event.key() in self.keys and self.keys[event.key()] != target_value:
            self.keys[event.key()] = target_value
            GUINode().get_logger().info(str(self.keys))
