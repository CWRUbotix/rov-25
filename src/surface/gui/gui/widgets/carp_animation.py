from pathlib import Path

from typing import List

from ament_index_python.packages import get_package_share_directory
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap, QPainter, QPen, QColor, QFont
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget, QPushButton, QComboBox

class AnimationLabel(QLabel):
    def __init__(self, year_list: List[int], interval=1000):
        super().__init__()
        self.year = 2015
        self.year_list = year_list

        map_path = str(Path(get_package_share_directory('gui')) / 'images' / 'Illinois River Map 0.png')
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


    def update_frame(self):
        region = 0
        for year in self.year_list:
            if self.year >= year:
                region += 1

        map_path = str(Path(get_package_share_directory('gui')) / 'images' / ('Illinois River Map ' + str(region) + '.png'))

        map_pixmap = QPixmap(map_path)

        painter = QPainter(map_pixmap)
        pen = QPen(QColor("black"))
        painter.setFont(QFont("Arial", 20))
        painter.setPen(pen)
        painter.drawText(20, 40, str(self.year))
        painter.end()
        self.setPixmap(
            map_pixmap.scaled(
                750,
                750,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )

        self.year = ((self.year - 2016 + 1) % 10) + 2016

class CarpAnimation(QWidget):
    def __init__(self) -> None:
        super().__init__()

        self.root_layout = QHBoxLayout()

        input_layout = QVBoxLayout()

        self.dropdown_list: List[QComboBox] = []

        for _ in range(5):
            dropdown = QComboBox()
            options = list(map(str,range(2016,2026))) # Generate list of years [2016,2025]
            dropdown.addItems(options)
            self.dropdown_list.append(dropdown)
            input_layout.addWidget(dropdown)

        self.setLayout(self.root_layout)

        show_button = QPushButton("Show Animation", None)

        input_layout.addWidget(show_button)
        
        self.root_layout.addLayout(input_layout)

        show_button.clicked.connect(self.showAnimation)
        # root_layout.addSpacing(10)

        # text_vbox = QVBoxLayout()
        # title_label = QLabel('Explorer Team 25 - CWRUbotix')
        # title_label.setStyleSheet('QLabel { font-size: 35px; }')
        # title_label.setAlignment(Qt.AlignmentFlag.AlignAbsolute)
        # text_vbox.addWidget(title_label)

        # subtitle_label = QLabel('Case Western Reserve University - Cleveland, Ohio')
        # subtitle_label.setStyleSheet('QLabel { font-size: 20px; }')
        # subtitle_label.setAlignment(Qt.AlignmentFlag.AlignAbsolute)
        # text_vbox.addWidget(subtitle_label)

        # root_layout.addLayout(text_vbox)
    
    def showAnimation(self):
        self.year_list = [int(dropdown.currentText()) for dropdown in self.dropdown_list]
        animation_label = AnimationLabel(self.year_list)
        self.root_layout.addWidget(animation_label)
        

        
