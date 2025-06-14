from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from enum import Enum, IntEnum
from math import atan, sqrt, tan
from typing import Generic, TypeGuard, TypeVar, override

from PyQt6.QtCore import QRect, Qt, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QColor, QFont, QImage, QKeyEvent, QMouseEvent, QPainter, QPixmap
from PyQt6.QtWidgets import (
    QGridLayout,
    QHBoxLayout,
    QLabel,
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

FRAME_WIDTH = 816
FRAME_HEIGHT = 510
# FRAME_WIDTH = 1280
# FRAME_HEIGHT = 800

ZOOMED_WIDGET_SIZE = 405
ZOOM_DEFAULT_IDX = 2
ZOOMED_VIEWPORT_SIZES = (27, 45, 81, 135, 405)  # Odd factors of 405

PADDING = 200

POINTS_PER_EYE = 2

BASELINE_MM = 60.6
TUBE_RADIUS_MM = 40

DIVISION_SAFETY = 0.0001

BLACK = QColor(Qt.GlobalColor.black)


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


T = TypeVar('T', int, float)


@dataclass
class Point2D(Generic[T]):
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


def has_all_points(
    key_points: dict[Eye, list[Point2D[int] | None]],
) -> TypeGuard['dict[Eye, list[Point2D[int]]]']:
    return all(
        len(key_points[eye]) == POINTS_PER_EYE
        and all(point is not None for point in key_points[eye])
        for eye in Eye
    )


class ShipwreckTab(QWidget):
    click_left_signal = pyqtSignal(QMouseEvent)
    click_right_signal = pyqtSignal(QMouseEvent)

    intrinsics_left_signal = pyqtSignal(Intrinsics)
    intrinsics_right_signal = pyqtSignal(Intrinsics)

    def __init__(self) -> None:
        super().__init__()

        self.img_points: dict[Eye, list[Point2D[int] | None]] = {
            Eye.LEFT: [None, None],
            Eye.RIGHT: [None, None],
        }

        self.keys: dict[int, bool] = {
            Qt.Key.Key_1.value: False,
            Qt.Key.Key_2.value: False,
            Qt.Key.Key_3.value: False,
            Qt.Key.Key_4.value: False,
        }

        self.intrinsics_left: Intrinsics | None = None
        self.intrinsics_right: Intrinsics | None = None

        self.crosshair = Crosshair.Dot
        self.viewport_zoom_level = ZOOM_DEFAULT_IDX

        tabs = QTabWidget()
        tabs.addTab(self.make_coarse_tab(), 'Coarse')
        tabs.addTab(self.make_fine_tab(), 'Fine')

        root_layout = QVBoxLayout()
        root_layout.addWidget(tabs)
        self.setLayout(root_layout)

        GUINode().create_signal_subscription(
            Intrinsics, 'luxonis_left_intrinsics', self.intrinsics_left_signal
        )
        GUINode().create_signal_subscription(
            Intrinsics, 'luxonis_right_intrinsics', self.intrinsics_right_signal
        )

        self.intrinsics_left_signal.connect(self.intrinsics_left_slot)
        self.intrinsics_right_signal.connect(self.intrinsics_right_slot)

        # Make sure we can get keyboard
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def make_coarse_tab(self) -> QWidget:
        self.click_left_signal.connect(self.click_left_slot)
        self.click_right_signal.connect(self.click_right_slot)

        cam_layout = QHBoxLayout()

        # TODO: RESET THIS TO ACTUAL LUXONIS CAM ONCE IT'S WORKING AGAIN
        self.eye_widgets = {
            Eye.LEFT: SwitchableVideoWidget(
                (
                    CameraDescription(
                        CameraType.DEPTH,
                        'rect_left/image_raw',
                        'Stream stopped',
                        FRAME_WIDTH,
                        FRAME_HEIGHT,
                    ),
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
                make_label=lambda: ClickableLabel(self.click_left_signal),
            ),
            Eye.RIGHT: SwitchableVideoWidget(
                (
                    CameraDescription(
                        CameraType.DEPTH,
                        'rect_right/image_raw',
                        'Stream stopped',
                        FRAME_WIDTH,
                        FRAME_HEIGHT,
                    ),
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
                make_label=lambda: ClickableLabel(self.click_right_signal),
            ),
        }

        for eye_widget in self.eye_widgets.values():
            cam_layout.addWidget(eye_widget)

        coarse_tab = QWidget()
        coarse_tab.setLayout(cam_layout)

        return coarse_tab

    def make_fine_tab(self) -> QWidget:
        self.zoomed_eye_widgets = {
            Eye.LEFT: (
                VideoWidget(
                    CameraDescription(
                        CameraType.QPIXMAP,
                        '',
                        'Left 1',
                        ZOOMED_WIDGET_SIZE,
                        ZOOMED_WIDGET_SIZE,
                    )
                ),
                VideoWidget(
                    CameraDescription(
                        CameraType.QPIXMAP,
                        '',
                        'Left 2',
                        ZOOMED_WIDGET_SIZE,
                        ZOOMED_WIDGET_SIZE,
                    )
                ),
            ),
            Eye.RIGHT: (
                VideoWidget(
                    CameraDescription(
                        CameraType.QPIXMAP,
                        '',
                        'Right 1',
                        ZOOMED_WIDGET_SIZE,
                        ZOOMED_WIDGET_SIZE,
                    )
                ),
                VideoWidget(
                    CameraDescription(
                        CameraType.QPIXMAP,
                        '',
                        'Right 2',
                        ZOOMED_WIDGET_SIZE,
                        ZOOMED_WIDGET_SIZE,
                    )
                ),
            ),
        }

        zoom_layout = QHBoxLayout()
        for widget_pair in self.zoomed_eye_widgets.values():
            for widget in widget_pair:
                zoom_layout.addWidget(widget)

        data_layout = QGridLayout()

        self.img_points_label = QLabel('No 2D points')
        self.focal_length_label = QLabel('No focal length')
        self.world_points_label = QLabel('No 3D points')
        self.underwater_world_points_label = QLabel('No 3D underwater points')
        self.length_label = QLabel('No length')
        self.underwater_length_label = QLabel('No underwater length')
        bold_font = QFont()
        bold_font.setBold(True)
        self.length_label.setFont(bold_font)
        self.underwater_length_label.setFont(bold_font)

        data_layout.addWidget(self.img_points_label, 0, 0)
        data_layout.addWidget(self.focal_length_label, 0, 1)
        data_layout.addWidget(self.world_points_label, 1, 0)
        data_layout.addWidget(self.length_label, 1, 1)
        data_layout.addWidget(self.underwater_world_points_label, 2, 0)
        data_layout.addWidget(self.underwater_length_label, 2, 1)

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

        focal_left_mm = Point2D(
            ShipwreckTab.px_to_mm(self.intrinsics_left.fx),
            ShipwreckTab.px_to_mm(self.intrinsics_left.fy),
        )
        focal_right_mm = Point2D(
            ShipwreckTab.px_to_mm(self.intrinsics_right.fx),
            ShipwreckTab.px_to_mm(self.intrinsics_right.fy),
        )
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

    def set_point(
        self, x: int, y: int, eye_override: Eye | None = None, *, relative: bool = False
    ) -> None:
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

        point.x = min(max(point.x, 0), FRAME_WIDTH - 1)
        point.y = min(max(point.y, 0), FRAME_HEIGHT - 1)

        self.img_points[eye][idx] = point

        self.reload_zoomed_views(eyes=eye, idxs=idx)

        self.img_points_label.setText(
            f'2D (px): {
                ", ".join(["-".join([str(p) for p in pnts]) for pnts in self.img_points.values()])
            }'
        )

        self.calc_world_points()

    def reload_zoomed_views(
        self, eyes: Eye | Iterable[Eye] = (Eye.LEFT, Eye.RIGHT), idxs: int | Iterable[int] = (0, 1)
    ) -> None:
        if isinstance(eyes, Eye):
            eyes = (eyes,)
        if isinstance(idxs, int):
            idxs = (idxs,)

        for eye in eyes:
            for idx in idxs:
                point = self.img_points[eye][idx]

                if point is None:
                    continue

                viewport_size = ZOOMED_VIEWPORT_SIZES[self.viewport_zoom_level]
                rect = QRect(
                    max(point.x + PADDING - (viewport_size // 2), 0),
                    max(point.y + PADDING - (viewport_size // 2), 0),
                    viewport_size,
                    viewport_size,
                )
                pixmap = self.eye_widgets[eye].get_pixmap()
                viewport = ShipwreckTab.add_padding_to_pixmap(pixmap, PADDING).copy(rect)
                viewport = viewport.scaledToWidth(ZOOMED_WIDGET_SIZE)

                painter = QPainter(viewport)
                painter.setPen(QColor(255, 0, 0, 127))

                pixel_size = ZOOMED_WIDGET_SIZE // viewport_size
                if self.crosshair == Crosshair.Dot:
                    painter.drawRect(
                        ZOOMED_WIDGET_SIZE // 2 - pixel_size // 2,
                        ZOOMED_WIDGET_SIZE // 2 - pixel_size // 2,
                        pixel_size,
                        pixel_size,
                    )

                painter.end()

                self.zoomed_eye_widgets[eye][idx].set_pixmap(viewport)

    @staticmethod
    def add_padding_to_pixmap(pixmap: QPixmap, padding: int, color: QColor = BLACK) -> QPixmap:
        new_width = pixmap.width() + 2 * padding
        new_height = pixmap.height() + 2 * padding

        padded_image = QImage(new_width, new_height, QImage.Format.Format_ARGB32)
        padded_image.fill(color)

        painter = QPainter(padded_image)
        painter.drawPixmap(padding, padding, pixmap)
        painter.end()

        return QPixmap.fromImage(padded_image)

    def get_key_point_eye_idx(self) -> tuple[Eye, int] | None:
        for key, eye_and_idx in KEYS_TO_POINT_IDX.items():
            if self.keys[key]:
                return eye_and_idx
        return None

    def calc_world_points(self) -> None:
        if (
            not has_all_points(self.img_points)
            or self.intrinsics_left is None
            or self.intrinsics_right is None
            or not isinstance(self.intrinsics_left.fx, float)
        ):
            return

        # TODO: using x focal len of left eye for both rn
        # Don't use focal lengths in real-world units unless
        #  you convert image space x/y to real world units too

        world_points, length = self.solve_stereo_projection(
            self.intrinsics_left.fx, (BASELINE_MM, BASELINE_MM), self.img_points
        )
        self.world_points_label.setText(
            f'3D (mm): {"\t".join([str(point) for point in world_points])}'
        )
        self.length_label.setText(f'Length (mm): {length}')

        ds: dict[Eye, list[float]] = {Eye.LEFT: [], Eye.RIGHT: []}
        thetas: dict[Eye, list[float]] = {Eye.LEFT: [], Eye.RIGHT: []}
        uw_img_points: dict[Eye, list[Point2D[float]]] = {Eye.LEFT: [], Eye.RIGHT: []}
        uw_baselines: list[float] = []

        for i in (0, 1):
            for eye in (Eye.LEFT, Eye.RIGHT):
                ds[eye].append(
                    (TUBE_RADIUS_MM / self.intrinsics_left.fx) * self.img_points[eye][i].x
                )
                thetas[eye].append(atan(self.img_points[eye][i].x / self.intrinsics_left.fx))
                # ys are unaffected by distortion
                uw_img_points[eye].append(
                    Point2D(
                        self.intrinsics_left.fx * tan(thetas[eye][i]),
                        float(self.img_points[eye][i].y),
                    )
                )
            uw_baselines.append(BASELINE_MM + ds[Eye.LEFT][i] - ds[Eye.RIGHT][i])

        uw_world_points, uw_length = ShipwreckTab.solve_stereo_projection(
            self.intrinsics_left.fx, uw_baselines, uw_img_points
        )
        self.underwater_world_points_label.setText(
            f'3D (mm): {"\t".join([str(point) for point in uw_world_points])}'
        )
        self.underwater_length_label.setText(f'Length (mm): {uw_length}')

    @staticmethod
    def solve_stereo_projection(
        f_px: float,
        baselines_mm: Sequence[float],
        img_points: dict[Eye, list[Point2D[int]]] | dict[Eye, list[Point2D[float]]],
    ) -> tuple[list[Point3D], float]:
        if len(baselines_mm) != len(img_points[Eye.LEFT]) or len(img_points[Eye.LEFT]) != len(
            img_points[Eye.RIGHT]
        ):
            raise IndexError(
                'Failed to solve stereo projection, lengths not the same: '
                f'{len(baselines_mm)} != {len(img_points[Eye.LEFT])} or '
                f'{len(img_points[Eye.LEFT])} != '
                '{len(img_points[Eye.RIGHT])}'
            )

        zs = []
        xs = []
        ys = []

        for i in range(len(img_points[Eye.LEFT])):
            disparity = img_points[Eye.LEFT][i].x - img_points[Eye.RIGHT][i].x
            zs.append(f_px * baselines_mm[i] / (disparity if disparity != 0 else DIVISION_SAFETY))
            left_point = img_points[Eye.LEFT][i]
            xs.append(left_point.x * zs[i] / f_px)
            ys.append(left_point.y * zs[i] / f_px)

        world_points = [Point3D(x, y, z) for x, y, z in zip(xs, ys, zs, strict=True)]

        length = sqrt(
            (world_points[0].x - world_points[1].x) ** 2
            + (world_points[0].y - world_points[1].y) ** 2
            + (world_points[0].z - world_points[1].z) ** 2
        )

        return (world_points, length)

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
        elif event.key() == Qt.Key.Key_Space:
            self.crosshair = Crosshair.Dot if self.crosshair == Crosshair.Empty else Crosshair.Empty
            self.reload_zoomed_views()
        elif event.key() == Qt.Key.Key_Minus:
            self.viewport_zoom_level = min(
                self.viewport_zoom_level + 1, len(ZOOMED_VIEWPORT_SIZES) - 1
            )
            self.reload_zoomed_views()
        elif event.key() == Qt.Key.Key_Plus:
            self.viewport_zoom_level = max(self.viewport_zoom_level - 1, 0)
            self.reload_zoomed_views()
        elif event.key() == Qt.Key.Key_Equal:
            self.viewport_zoom_level = ZOOM_DEFAULT_IDX
            self.reload_zoomed_views()

    def keyReleaseEvent(self, event: QKeyEvent | None) -> None:  # noqa: N802
        self.handle_key_event(event, target_value=False)

    def handle_key_event(self, event: QKeyEvent | None, *, target_value: bool) -> None:
        # Shouldn't ever happen, just type narrowing
        if event is None:
            return

        if (
            not event.isAutoRepeat()
            and event.key() in self.keys
            and self.keys[event.key()] != target_value
        ):
            self.keys[event.key()] = target_value
