from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QWidget

from gui.gui_node import GUINode
from gui.styles.custom_styles import WidgetState
from gui.widgets.circle import CircleIndicator
from rov_msgs.msg import Manip


class ManipStatus(QWidget):
    signal = pyqtSignal(Manip)

    def __init__(self, manip_name: str) -> None:
        super().__init__()

        self.manip_id = manip_name

        self.signal.connect(self.manip_callback)
        GUINode().create_signal_subscription(Manip, 'manipulator_control', self.signal)

        root_layout = QHBoxLayout()

        self.indicator_circle = CircleIndicator(radius=10)
        root_layout.addWidget(self.indicator_circle)

        self.label = QLabel('')
        font = QFont('Arial', 14)
        self.label.setFont(font)
        root_layout.addWidget(self.label)

        self.setLayout(root_layout)

        self.update_gui(manip_on=False)

    def update_gui(self, *, manip_on: bool) -> None:
        self.label.setText(f'{self.manip_id.capitalize()} manip {"ON" if manip_on else "OFF"}')
        self.indicator_circle.set_state(WidgetState.ON if manip_on else WidgetState.OFF)

    @pyqtSlot(Manip)
    def manip_callback(self, msg: Manip) -> None:
        if msg.manip_id == self.manip_id:
            self.update_gui(manip_on=not msg.activated)
