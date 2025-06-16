from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QColor, QFont, QPainter, QPen, QPixmap
from PyQt6.QtWidgets import QComboBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

START_YEAR = 2016
END_YEAR = 2025


class AnimationLabel(QLabel):
    def __init__(self, year_list: list[int], interval: int = 2000) -> None:
        super().__init__()
        self.year = START_YEAR
        self.year_list = year_list

        map_path = str(
            Path(get_package_share_directory('gui')) / 'images' / 'illinois_river_map_0.png'
        )
        map_pixmap = QPixmap(map_path)
        self.setPixmap(
            map_pixmap.scaled(
                750,
                750,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(interval)  # in milliseconds

    def update_frame(self) -> None:
        region = 0
        for year in self.year_list:
            if self.year >= year:
                region += 1

        map_path = str(
            Path(get_package_share_directory('gui'))
            / 'images'
            / ('illinois_river_map_' + str(region) + '.png')
        )

        map_pixmap = QPixmap(map_path)

        painter = QPainter(map_pixmap)
        pen = QPen(QColor('black'))
        painter.setFont(QFont('Arial', 50))
        painter.setPen(pen)
        painter.drawText(20, 60, str(self.year))
        painter.end()
        self.setPixmap(
            map_pixmap.scaled(
                750,
                750,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )

        self.year += 1
        self.year = min(self.year, END_YEAR)


class CarpAnimation(QWidget):
    def __init__(self) -> None:
        super().__init__()

        self.root_layout = QHBoxLayout()

        input_layout = QVBoxLayout()

        self.dropdown_list: list[QComboBox] = []

        for _ in range(5):
            dropdown = QComboBox()
            # List of years [2016,2025]
            options = list(map(str, range(2016, 2026)))
            options.append('9999')
            dropdown.addItems(options)
            dropdown.setCurrentIndex(len(options) - 1)
            self.dropdown_list.append(dropdown)
            input_layout.addWidget(dropdown)

        self.setLayout(self.root_layout)

        show_button = QPushButton('Show Animation', None)

        input_layout.addWidget(show_button)
        input_layout.addStretch()

        self.root_layout.addLayout(input_layout)

        show_button.clicked.connect(self.show_animation)

        self.animation_label: AnimationLabel | None = None

    def show_animation(self) -> None:
        self.root_layout.removeWidget(self.animation_label)
        self.year_list = [int(dropdown.currentText()) for dropdown in self.dropdown_list]
        self.animation_label = AnimationLabel(self.year_list)
        self.root_layout.addWidget(self.animation_label)
