from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QVBoxLayout, QWidget

from gui.widgets.carp_animation import CarpAnimation


class CarpModelTab(QWidget):
    def __init__(self) -> None:
        super().__init__()

        root_layout = QVBoxLayout()
        root_layout.addWidget(
            CarpAnimation(), alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft
        )
        self.setLayout(root_layout)
