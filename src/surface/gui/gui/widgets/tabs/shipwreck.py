from collections.abc import Callable
from dataclasses import dataclass
from math import sqrt
from typing import TypeGuard, override

from PyQt6.QtCore import QEvent, QObject, Qt, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QFont, QKeyEvent, QMouseEvent
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QScrollArea, QVBoxLayout, QWidget

from gui.gui_node import GUINode
from gui.widgets.video_widget import (
    CameraDescription,
    CameraManager,
    CameraType,
    ClickableLabel,
    SwitchableVideoWidget,
)
from rov_msgs.msg import Intrinsics
from rov_msgs.srv import CameraManage

FRAME_WIDTH = 921
FRAME_HEIGHT = 690

POINTS_PER_EYE = 2

BASELINE_MM = 60.6

@dataclass
class Point2D:
    x: float
    y: float

    @override
    def __str__(self) -> str:
        return f'({round(self.x, 3)}, {round(self.y, 3)})'

@dataclass
class Point3D:
    x: float
    y: float
    z: float

    @override
    def __str__(self) -> str:
        return f'({round(self.x, 3)}, {round(self.y, 3)}, {round(self.z, 3)})'

@dataclass
class KeyPoints[T: Point2D | Point3D | None]:
    left_eye: list[T]
    right_eye: list[T]

    @override
    def __str__(self) -> str:
        return f'{self.left_eye[0]}-{self.left_eye[1]} \t {self.right_eye[0]}-{self.right_eye[1]}'

def has_all_points[T: Point2D | Point3D | None](key_points: KeyPoints[T]) -> \
    TypeGuard['KeyPoints[Point2D | Point3D]']:
    return (
        all(point is not None for point in key_points.left_eye + key_points.right_eye) and
        len(key_points.left_eye) == POINTS_PER_EYE and
        len(key_points.right_eye) == POINTS_PER_EYE
    )

class ShipwreckTab(QWidget):
    click_left_signal = pyqtSignal(QMouseEvent)
    click_right_signal = pyqtSignal(QMouseEvent)

    intrinsics_left_signal = pyqtSignal(Intrinsics)
    intrinsics_right_signal = pyqtSignal(Intrinsics)

    def __init__(self) -> None:
        super().__init__()

        self.click_left_signal.connect(self.click_left_slot)
        self.click_right_signal.connect(self.click_right_slot)

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
            'switch_rect_stream',
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
            'switch_rect_stream',
            make_label=lambda: ClickableLabel(self.click_right_signal)
        )

        cam_layout.addWidget(left_eye)
        cam_layout.addWidget(right_eye)

        data_layout = QVBoxLayout()
        row_1 = QHBoxLayout()
        row_2 = QHBoxLayout()

        self.img_points_label = QLabel('No 2D points')
        self.focal_length_label = QLabel('No focal length')
        self.world_points_label = QLabel('No 3D points')
        self.length_label = QLabel('No length')
        bold_font = QFont()
        bold_font.setBold(True)
        self.length_label.setFont(bold_font)
        row_1.addWidget(self.img_points_label)
        row_1.addWidget(self.length_label)
        row_2.addWidget(self.focal_length_label)
        row_2.addWidget(self.world_points_label)

        data_layout.addLayout(row_1)
        data_layout.addLayout(row_2)

        scroll_layout = QVBoxLayout()
        scroll_layout.addLayout(cam_layout)
        scroll_layout.addLayout(data_layout)

        scroll = QScrollArea()
        scroll.setLayout(scroll_layout)
        scroll.setWidgetResizable(True)

        root_layout = QVBoxLayout()
        root_layout.addWidget(scroll)
        self.setLayout(root_layout)

        self.img_points = KeyPoints[Point2D | None]([None, None], [None, None])
        self.world_points: list[Point3D] = []

        self.keys: dict[int, bool] = {
            Qt.Key.Key_1.value: False,
            Qt.Key.Key_2.value: False,
        }

        GUINode().create_signal_subscription(Intrinsics, 'luxonis_left_intrinsics',
                                             self.intrinsics_left_signal)
        GUINode().create_signal_subscription(Intrinsics, 'luxonis_right_intrinsics',
                                             self.intrinsics_right_signal)

        self.intrinsics_left_signal.connect(self.intrinsics_left_slot)
        self.intrinsics_right_signal.connect(self.intrinsics_right_slot)

        self.intrinsics_left: Intrinsics | None = None
        self.intrinsics_right: Intrinsics | None = None

    @staticmethod
    def px_to_mm(px: float) -> float:
        # 3 um/px (https://docs.luxonis.com/hardware/sensors/OV9782)
        # / 1000 to get mm
        return px * 3 / 1000

    @pyqtSlot(Intrinsics)
    def intrinsics_left_slot(self, intrinsics: Intrinsics) -> None:
        self.intrinsics_left = intrinsics
        self.show_intrinsics()

    @pyqtSlot(Intrinsics)
    def intrinsics_right_slot(self, intrinsics: Intrinsics) -> None:
        self.intrinsics_right = intrinsics
        self.show_intrinsics()

    def show_intrinsics(self) -> None:
        if self.intrinsics_left is None or self.intrinsics_right is None:
            return

        focal_left_mm = Point2D(ShipwreckTab.px_to_mm(self.intrinsics_left.fx),
                                ShipwreckTab.px_to_mm(self.intrinsics_left.fy))
        focal_right_mm = Point2D(ShipwreckTab.px_to_mm(self.intrinsics_right.fx),
                                 ShipwreckTab.px_to_mm(self.intrinsics_right.fy))
        self.focal_length_label.setText(f'Focal lens (mm): {focal_left_mm} \t {focal_right_mm}')

    @pyqtSlot(QMouseEvent)
    def click_left_slot(self, event: QMouseEvent) -> None:
        self.click_eye(event, is_right_eye=False)

    @pyqtSlot(QMouseEvent)
    def click_right_slot(self, event: QMouseEvent) -> None:
        self.click_eye(event, is_right_eye=True)

    def click_eye(self, event: QMouseEvent, *, is_right_eye: bool) -> None:
        if self.keys[Qt.Key.Key_2]:
            point_idx = 1
        elif self.keys[Qt.Key.Key_1]:
            point_idx = 0
        else:
            return  # No key pressed, don't register point

        point = Point2D(event.pos().x(), event.pos().y())

        if is_right_eye:
            self.img_points.right_eye[point_idx] = point
        else:
            self.img_points.left_eye[point_idx] = point

        self.img_points_label.setText(f'2D (px): {self.img_points}')

        self.calc_world_points()

    def calc_world_points(self) -> None:
        if (not has_all_points(self.img_points) or
            self.intrinsics_left is None or self.intrinsics_right is None):
            return

        # TODO: using x focal len of left eye for both rn
        # Don't use focal lengths in real-world units unless
        #  you convert image space x/y to real world units too
        f = self.intrinsics_left.fx
        zs = []
        xs = []
        ys = []

        for i in (0, 1):
            zs.append(f * BASELINE_MM / (self.img_points.left_eye[i].x -
                                         self.img_points.right_eye[i].x))
            left_point = self.img_points.left_eye[i]
            xs.append(left_point.x * zs[i] / f)
            ys.append(left_point.y * zs[i] / f)

        self.world_points = [Point3D(x, y, z) for x, y, z in zip(xs, ys, zs, strict=True)]
        self.world_points_label.setText(f'3D (mm): {'\t'.join(
            [str(point) for point in self.world_points])}')

        length = sqrt((self.world_points[0].x - self.world_points[1].x) ** 2 +
                      (self.world_points[0].y - self.world_points[1].y) ** 2 +
                      (self.world_points[0].z - self.world_points[1].z) ** 2)
        self.length_label.setText(f'Length (mm): {length}')

    def keyPressEvent(self, event: QKeyEvent | None) -> None:  # noqa: N802
        self.handle_key_event(event, target_value=True)

    def keyReleaseEvent(self, event: QKeyEvent | None) -> None:  # noqa: N802
        self.handle_key_event(event, target_value=False)

    def handle_key_event(self, event: QKeyEvent | None, *, target_value: bool) -> None:
        # Shouldn't ever happen, just type narrowing
        if event is None:
            return

        if (not event.isAutoRepeat() and event.key() in self.keys
            and self.keys[event.key()] != target_value):
            self.keys[event.key()] = target_value
            GUINode().get_logger().info(str(self.keys))
