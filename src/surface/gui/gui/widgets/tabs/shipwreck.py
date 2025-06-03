from collections.abc import Callable
from dataclasses import dataclass
from enum import IntEnum
from math import sqrt
from typing import TypeGuard, override

from PyQt6.QtCore import QEvent, QObject, QRect, Qt, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QFont, QKeyEvent, QMouseEvent
from PyQt6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from gui.gui_node import GUINode
from gui.widgets.video_widget import (
    CameraDescription,
    CameraManager,
    CameraType,
    ClickableLabel,
    SwitchableVideoWidget,
    VideoWidget,
)
from rov_msgs.msg import Intrinsics
from rov_msgs.srv import CameraManage

FRAME_WIDTH = 821
FRAME_HEIGHT = 791

ZOOMED_FRAME_WIDTH = 411
ZOOMED_FRAME_HEIGHT = 411

POINTS_PER_EYE = 2

BASELINE_MM = 60.6

class Eye(IntEnum):
    LEFT = 0
    RIGHT = 1

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

def has_all_points[T: Point2D | None](key_points: dict[Eye, list[T]]) -> \
    TypeGuard['dict[Eye, list[Point2D]]']:
    return all(len(key_points[eye]) == POINTS_PER_EYE and
                all(point is not None for point in key_points[eye])
                for eye in Eye)

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

        self.eye_widgets = {
            Eye.LEFT: SwitchableVideoWidget(
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
            ),
            Eye.RIGHT: SwitchableVideoWidget(
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
        }

        for eye_widget in self.eye_widgets.values():
            cam_layout.addWidget(eye_widget)

        zoom_indicator_toggle = QPushButton('Toggle indicator')
        self.zoomed_eye_widgets = {
            Eye.LEFT: (
                VideoWidget(CameraDescription(
                    CameraType.QPIXMAP,
                    '', 'Left 1',
                    ZOOMED_FRAME_WIDTH,
                    ZOOMED_FRAME_HEIGHT,
                )),
                VideoWidget(CameraDescription(
                    CameraType.QPIXMAP,
                    '', 'Left 2',
                    ZOOMED_FRAME_WIDTH,
                    ZOOMED_FRAME_HEIGHT,
                )),
            ),
            Eye.RIGHT: (
                VideoWidget(CameraDescription(
                    CameraType.QPIXMAP,
                    '', 'Right 1',
                    ZOOMED_FRAME_WIDTH,
                    ZOOMED_FRAME_HEIGHT,
                )),
                VideoWidget(CameraDescription(
                    CameraType.QPIXMAP,
                    '', 'Right 2',
                    ZOOMED_FRAME_WIDTH,
                    ZOOMED_FRAME_HEIGHT,
                ))
            )
        }

        zoom_layout = QVBoxLayout()
        zoom_frames_layout = QHBoxLayout()
        for widget_pair in self.zoomed_eye_widgets.values():
            for widget in widget_pair:
                zoom_frames_layout.addWidget(widget)
        zoom_layout.addLayout(zoom_frames_layout)
        zoom_layout.addWidget(zoom_indicator_toggle)

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

        coarse_tab = QWidget()
        coarse_tab.setLayout(cam_layout)

        fine_tab = QWidget()
        fine_tab_layout = QVBoxLayout()
        fine_tab_layout.addLayout(zoom_layout)
        fine_tab_layout.addLayout(data_layout)
        fine_tab.setLayout(fine_tab_layout)

        tabs = QTabWidget()
        tabs.addTab(coarse_tab, 'Coarse')
        tabs.addTab(fine_tab, 'Fine')

        root_layout = QVBoxLayout()
        root_layout.addWidget(tabs)
        self.setLayout(root_layout)

        self.img_points: dict[Eye, list[Point2D | None]] = {
            Eye.LEFT: [None, None],
            Eye.RIGHT: [None, None]
        }
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
        self.click_eye(event, Eye.LEFT)

    @pyqtSlot(QMouseEvent)
    def click_right_slot(self, event: QMouseEvent) -> None:
        self.click_eye(event, Eye.RIGHT)

    def click_eye(self, event: QMouseEvent, eye: Eye) -> None:
        if self.keys[Qt.Key.Key_2]:
            point_idx = 1
        elif self.keys[Qt.Key.Key_1]:
            point_idx = 0
        else:
            return  # No key pressed, don't register point

        x = event.pos().x()
        y = event.pos().y()
        point = Point2D(x, y)

        self.img_points[eye][point_idx] = point
        rect = QRect(
            max(x - 50, 0),
            max(y - 50, 0),
            100,
            100
        )
        cropped = self.eye_widgets[eye].get_pixmap().copy(rect).scaledToWidth(ZOOMED_FRAME_WIDTH)
        self.zoomed_eye_widgets[eye][point_idx].set_pixmap(cropped)

        self.img_points_label.setText(f'2D (px): {
            ", ".join(["-".join([str(p) for p in pnts]) for pnts in self.img_points.values()])}')

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
            zs.append(f * BASELINE_MM / (self.img_points[Eye.LEFT][i].x -
                                         self.img_points[Eye.RIGHT][i].x))
            left_point = self.img_points[Eye.LEFT][i]
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
