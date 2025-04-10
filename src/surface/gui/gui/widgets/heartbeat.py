from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

from gui.gui_node import GUINode
from gui.styles.custom_styles import WidgetState
from gui.widgets.circle import CircleIndicator
from rov_msgs.msg import VehicleState


class HeartbeatWidget(QWidget):
    signal = pyqtSignal(VehicleState)

    def __init__(self) -> None:
        super().__init__()

        self.signal.connect(self.refresh)
        GUINode().create_signal_subscription(VehicleState, 'vehicle_state_event', self.signal)
        # Create a latch variable
        self.warning_msg_latch: bool = False

        heartbeat_layout = QVBoxLayout()

        font = QFont('Arial', 14)

        pi_status_layout = QHBoxLayout()
        self.pi_indicator = QLabel('No Pi Status')
        self.pi_indicator.setFont(font)
        pi_status_layout.addWidget(self.pi_indicator)
        self.pi_indicator_circle = CircleIndicator(radius=10)
        pi_status_layout.addWidget(self.pi_indicator_circle)
        heartbeat_layout.addLayout(pi_status_layout)

        ardusub_status_layout = QHBoxLayout()
        self.ardusub_indicator = QLabel('No Ardusub Status')
        self.ardusub_indicator.setFont(font)
        ardusub_status_layout.addWidget(self.ardusub_indicator)
        self.ardusub_indicator_circle = CircleIndicator(radius=10)
        ardusub_status_layout.addWidget(self.ardusub_indicator_circle)
        heartbeat_layout.addLayout(ardusub_status_layout)

        self.setLayout(heartbeat_layout)

    @pyqtSlot(VehicleState)
    def refresh(self, msg: VehicleState) -> None:
        if msg.pi_connected:
            self.pi_indicator.setText('Pi Connected')
            self.pi_indicator_circle.set_state(WidgetState.ON)
        else:
            self.pi_indicator.setText('Pi Disconnected')
            self.pi_indicator_circle.set_state(WidgetState.OFF)

        if msg.ardusub_connected:
            self.ardusub_indicator.setText('Ardusub Connected')
            self.ardusub_indicator_circle.set_state(WidgetState.ON)
        else:
            self.ardusub_indicator.setText('Ardusub Disconnected')
            self.ardusub_indicator_circle.set_state(WidgetState.OFF)
