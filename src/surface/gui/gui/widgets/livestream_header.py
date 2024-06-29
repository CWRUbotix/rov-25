from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget
from PyQt6.QtGui import QPixmap

from ament_index_python.packages import get_package_share_directory
import os


class LivestreamHeader(QWidget):
    def __init__(self) -> None:
        super().__init__()

        root_layout = QHBoxLayout()
        self.setLayout(root_layout)

        logo_path = os.path.join(
            get_package_share_directory("gui"), "images", "CWRUbotix Logo.png"
        )

        logo_pixmap = QPixmap(logo_path)

        logo_label = QLabel()
        logo_label.setPixmap(
            logo_pixmap.scaled(100, 100, Qt.AspectRatioMode.KeepAspectRatio,
                               Qt.TransformationMode.SmoothTransformation)
        )

        root_layout.addWidget(logo_label)
        root_layout.addSpacing(10)

        text_vbox = QVBoxLayout()
        title_label = QLabel("Explorer Team 25 - CWRUbotix")
        title_label.setStyleSheet('QLabel { font-size: 35px; }')
        title_label.setAlignment(Qt.AlignmentFlag.AlignAbsolute)
        text_vbox.addWidget(title_label)

        subtitle_label = QLabel("Case Western Reserve University - Cleveland, Ohio")
        subtitle_label.setStyleSheet('QLabel { font-size: 20px; }')
        subtitle_label.setAlignment(Qt.AlignmentFlag.AlignAbsolute)
        text_vbox.addWidget(subtitle_label)

        root_layout.addLayout(text_vbox)
