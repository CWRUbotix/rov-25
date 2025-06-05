from collections.abc import Callable, Iterable
from dataclasses import dataclass
from enum import Enum, IntEnum
from math import sqrt
from typing import TypeGuard, override

from PyQt6.QtCore import QEvent, QObject, QRect, Qt, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QColor, QFont, QKeyEvent, QMouseEvent, QPainter
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

ZOOMED_WIDGET_SIZE = 400
ZOOMED_VIEWPORT_SIZE = 100

POINTS_PER_EYE = 2

BASELINE_MM = 60.6

class Eye(IntEnum):
    LEFT = 0
    RIGHT = 1

class Crosshair(Enum):
    Empty = 0
    Dot = 1

KEYS_TO_POINT_IDX = {
    Qt.Key.Key_1: (Eye.LEFT, 0),
    Qt.Key.Key_2: (Eye.LEFT, 1),
    Qt.Key.Key_3: (Eye.RIGHT, 0),
    Qt.Key.Key_4: (Eye.RIGHT, 1),
}

@dataclass
class Point2D[T: (int, float)]:
    x: T
    y: T

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

def has_all_points[T: Point2D[int] | None](key_points: dict[Eye, list[T]]) -> \
    TypeGuard['dict[Eye, list[Point2D[int]]]']:
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

        self.img_points: dict[Eye, list[Point2D[int] | None]] = {
            Eye.LEFT: [None, None],
            Eye.RIGHT: [None, None]
        }
        self.world_points: list[Point3D] = []

        self.keys: dict[int, bool] = {
            Qt.Key.Key_1.value: False,
            Qt.Key.Key_2.value: False,
            Qt.Key.Key_3.value: False,
            Qt.Key.Key_4.value: False,
        }

        self.intrinsics_left: Intrinsics | None = None
        self.intrinsics_right: Intrinsics | None = None

        self.crosshair = Crosshair.Dot
        self.viewport_zoom_level = 0

        tabs = QTabWidget()
        tabs.addTab(self.make_coarse_tab(), 'Coarse')
        tabs.addTab(self.make_fine_tab(), 'Fine')

        root_layout = QVBoxLayout()
        root_layout.addWidget(tabs)
        self.setLayout(root_layout)

        GUINode().create_signal_subscription(Intrinsics, 'luxonis_left_intrinsics',
                                             self.intrinsics_left_signal)
        GUINode().create_signal_subscription(Intrinsics, 'luxonis_right_intrinsics',
                                             self.intrinsics_right_signal)

        self.intrinsics_left_signal.connect(self.intrinsics_left_slot)
        self.intrinsics_right_signal.connect(self.intrinsics_right_slot)

    def make_coarse_tab(self) -> QWidget:
        self.click_left_signal.connect(self.click_left_slot)
        self.click_right_signal.connect(self.click_right_slot)

        cam_layout = QHBoxLayout()

        # TODO: RESET THIS TO ACTUAL LUXONIS CAM ONCE IT'S WORKING AGAIN
        self.eye_widgets = {
            Eye.LEFT: SwitchableVideoWidget(
                (
                    CameraDescription(CameraType.DEPTH, 'rect_left/image_raw', 'Stream stopped',
                                    FRAME_WIDTH, FRAME_HEIGHT),
                    CameraDescription(
                        CameraType.ETHERNET,
                        'cam0/image_raw',
                        'Dual Left Eye',
                        FRAME_WIDTH,
                        FRAME_HEIGHT,
                        # CameraManager('manage_luxonis', CameraManage.Request.LUX_LEFT_RECT),
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
                        CameraType.ETHERNET,
                        'cam1/image_raw',
                        'Dual Right Eye',
                        FRAME_WIDTH,
                        FRAME_HEIGHT,
                        # CameraManager('manage_luxonis', CameraManage.Request.LUX_RIGHT_RECT),
                    ),
                ),
                'switch_rect_stream',
                make_label=lambda: ClickableLabel(self.click_right_signal)
            )
        }

        for eye_widget in self.eye_widgets.values():
            cam_layout.addWidget(eye_widget)

        coarse_tab = QWidget()
        coarse_tab.setLayout(cam_layout)

        return coarse_tab

    def make_fine_tab(self) -> QWidget:
        self.zoomed_eye_widgets = {
            Eye.LEFT: (
                VideoWidget(CameraDescription(
                    CameraType.QPIXMAP,
                    '', 'Left 1',
                    ZOOMED_WIDGET_SIZE,
                    ZOOMED_VIEWPORT_SIZE,
                )),
                VideoWidget(CameraDescription(
                    CameraType.QPIXMAP,
                    '', 'Left 2',
                    ZOOMED_WIDGET_SIZE,
                    ZOOMED_VIEWPORT_SIZE,
                )),
            ),
            Eye.RIGHT: (
                VideoWidget(CameraDescription(
                    CameraType.QPIXMAP,
                    '', 'Right 1',
                    ZOOMED_WIDGET_SIZE,
                    ZOOMED_VIEWPORT_SIZE,
                )),
                VideoWidget(CameraDescription(
                    CameraType.QPIXMAP,
                    '', 'Right 2',
                    ZOOMED_WIDGET_SIZE,
                    ZOOMED_VIEWPORT_SIZE,
                ))
            )
        }

        zoom_layout = QHBoxLayout()
        for widget_pair in self.zoomed_eye_widgets.values():
            for widget in widget_pair:
                zoom_layout.addWidget(widget)

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

        fine_tab = QWidget()
        fine_tab_layout = QVBoxLayout()
        fine_tab_layout.addLayout(zoom_layout)
        fine_tab_layout.addLayout(data_layout)
        fine_tab.setLayout(fine_tab_layout)

        return fine_tab

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
        x = event.pos().x()
        y = event.pos().y()

        self.set_point(x, y, eye_override=eye)

    def set_point(self, x: int, y: int, eye_override: Eye | None = None,
                  *, relative: bool = False) -> None:
        eye_and_idx = self.get_key_point_eye_idx()

        if eye_and_idx is None:
            return  # No number key pressed, don't register point

        eye, idx = eye_and_idx

        if eye_override is not None and eye != eye_override:
            return  # Clicked the wrong image, ignore

        if relative:
            point = self.img_points[eye][idx]
            if point is None:
                return  # No point exists yet, can't move it
            point.x += x
            point.y += y
        else:
            point = Point2D(x, y)

        self.img_points[eye][idx] = point

        self.reload_zoomed_views(eyes=eye, idxs=idx)

        self.img_points_label.setText(f'2D (px): {
            ", ".join(["-".join([str(p) for p in pnts]) for pnts in self.img_points.values()])}')

        self.calc_world_points()

    def reload_zoomed_views(self, eyes: Eye | Iterable[Eye] = Eye,
                            idxs: int | Iterable[int] = (0, 1)) -> None:
        if isinstance(eyes, Eye):
            eyes = (eyes,)
        if isinstance(idxs, int):
            idxs = (idxs,)

        for eye in eyes:
            for idx in idxs:
                point = self.img_points[eye][idx]

                if point is None:
                    continue

                rect = QRect(
                    max(point.x - 50, 0),
                    max(point.y - 50, 0),
                    ZOOMED_VIEWPORT_SIZE * (2 ** self.viewport_zoom_level),
                    ZOOMED_VIEWPORT_SIZE * (2 ** self.viewport_zoom_level)
                )
                viewport = self.eye_widgets[eye].get_pixmap().copy(rect)
                viewport = viewport.scaledToWidth(ZOOMED_WIDGET_SIZE)

                painter = QPainter(viewport)
                painter.setPen(QColor(255, 0, 0, 127))

                pixel_size = ZOOMED_WIDGET_SIZE // ZOOMED_VIEWPORT_SIZE
                if self.crosshair == Crosshair.Dot:
                    painter.drawRect(
                        ZOOMED_WIDGET_SIZE // 2 - pixel_size,
                        ZOOMED_WIDGET_SIZE // 2 - pixel_size,
                        pixel_size,
                        pixel_size
                    )

                self.zoomed_eye_widgets[eye][idx].set_pixmap(viewport)

    def get_key_point_eye_idx(self) -> tuple[Eye, int] | None:
        for key, eye_and_idx in KEYS_TO_POINT_IDX.items():
            if self.keys[key]:
                return eye_and_idx
        return None

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

        # Shouldn't ever happen, just type narrowing
        if event is None:
            return

        if event.key() == Qt.Key.Key_Left:
            self.set_point(-1, 0, relative=True)
        elif event.key() == Qt.Key.Key_Right:
            self.set_point(1, 0, relative=True)
        elif event.key() == Qt.Key.Key_Up:
            self.set_point(0, -1, relative=True)
        elif event.key() == Qt.Key.Key_Down:
            self.set_point(0, 1, relative=True)
        elif event.key() == Qt.Key.Key_Period:
            self.crosshair = Crosshair.Dot if self.crosshair == Crosshair.Empty else Crosshair.Empty
            self.reload_zoomed_views()
        elif event.key() == Qt.Key.Key_Minus:
            self.viewport_zoom_level -= 1
            self.reload_zoomed_views()
        elif event.key() == Qt.Key.Key_Plus:
            self.viewport_zoom_level += 1
            self.reload_zoomed_views()
        elif event.key() == Qt.Key.Key_Equal:
            self.viewport_zoom_level = 0
            self.reload_zoomed_views()

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
