from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QHBoxLayout, QVBoxLayout, QWidget

from gui.widgets.arm import Arm
from gui.widgets.heartbeat import HeartbeatWidget
from gui.widgets.ip_widget import IPWidget
from gui.widgets.logger import Logger


class GeneralDebugTab(QWidget):
    def __init__(self) -> None:
        super().__init__()

        top_bar = QHBoxLayout()
        top_bar.addWidget(
            IPWidget(), alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft
        )

        top_bar.addWidget(
            HeartbeatWidget(), alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft
        )

        right_bar = QVBoxLayout()
        # TODO: Update ThrusterTester to use Mavlink instead of Mavros
        # right_bar.addWidget(ThrusterTester())
        right_bar.addWidget(Arm())
        right_bar.setAlignment(Qt.AlignmentFlag.AlignRight)

        top_bar.addStretch(2)
        top_bar.addLayout(right_bar)

        root_layout = QVBoxLayout()
        root_layout.addLayout(top_bar)
        root_layout.addStretch()
        root_layout.addWidget(Logger())
        self.setLayout(root_layout)
