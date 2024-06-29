from collections import deque

from gui.gui_nodes.event_nodes.subscriber import GUIEventSubscriber
from rov_msgs.msg import Temperature
from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import (QLabel, QLineEdit, QPushButton,
                             QVBoxLayout, QWidget)

MIN_TEMP_C = 0
MAX_TEMP_C = 200
QUEUE_LEN = 5


class TemperatureSensor(QWidget):
    temperature_reading_signal: pyqtSignal = pyqtSignal(Temperature)

    def __init__(self) -> None:
        super().__init__()

        self.temperature_reading_signal.connect(self.temperature_received)
        self.temp_subscriber: GUIEventSubscriber = GUIEventSubscriber(
            Temperature,
            "temperature",
            self.temperature_reading_signal
        )

        self.temps: deque[float] = deque(maxlen=QUEUE_LEN)
        self.offset = 0.0

        root_layout = QVBoxLayout()
        self.setLayout(root_layout)

        self.ave_temp_label = QLabel()
        self.ave_temp_label.setText('Waiting for temp...')

        self.offset_field = QLineEdit()
        self.offset_field.setPlaceholderText('Enter offset (C)')

        self.offset_button = QPushButton('Set offset')
        self.offset_button.clicked.connect(self.set_offset)

        root_layout.addWidget(self.ave_temp_label)
        root_layout.addWidget(self.offset_field)
        root_layout.addWidget(self.offset_button)

    def set_offset(self) -> None:
        offset_text = self.offset_field.text()

        if offset_text == '':
            offset_text = '0'

        try:
            self.offset = float(offset_text)
            self.offset_button.setText('Offset applied')
        except ValueError:
            self.offset_button.setText('Illegal')
            return

    @pyqtSlot(Temperature)
    def temperature_received(self, msg: Temperature) -> None:
        if MIN_TEMP_C <= msg.reading <= MAX_TEMP_C:
            self.temps.append(msg.reading)

            ave = sum(self.temps) / QUEUE_LEN
            offset_ave = ave + self.offset
            self.ave_temp_label.setText(f'{round(offset_ave, 4)}\t°C')

            self.offset_button.setText('Set offset')
