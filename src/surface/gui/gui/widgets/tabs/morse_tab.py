from pathlib import Path
from threading import Thread
from time import sleep

from ament_index_python.packages import get_package_share_directory
from PyQt6.QtCore import Qt
from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtGui import QFont, QPixmap
from PyQt6.QtWidgets import (
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.qos import qos_profile_system_default

from gui.gui_node import GUINode
from gui.styles.custom_styles import WidgetState
from gui.widgets.circle import CircleIndicator
from rov_msgs.msg import Manip

DIT = '.'
DAH = '-'

TIME_UNIT_S = 0.5

MORSE_DICT = {
    'A': '.-',
    'B': '-...',
    'C': '-.-.',
    'D': '-..',
    'E': '.',
    'F': '..-.',
    'G': '--.',
    'H': '....',
    'I': '..',
    'J': '.---',
    'K': '-.-',
    'L': '.-..',
    'M': '--',
    'N': '-.',
    'O': '---',
    'P': '.--.',
    'Q': '--.-',
    'R': '.-.',
    'S': '...',
    'T': '-',
    'U': '..-',
    'V': '...-',
    'W': '.--',
    'X': '-..-',
    'Y': '-.--',
    'Z': '--..',
}

class MorseTab(QWidget):
    set_light_signal = pyqtSignal(bool)

    def __init__(self) -> None:
        super().__init__()

        self.manip_publisher = GUINode().create_publisher(
            Manip, 'manipulator_control', qos_profile_system_default
        )

        font = QFont('Arial', 50)

        root_layout = QHBoxLayout()
        left_layout = QGridLayout()
        right_layout = QVBoxLayout()
        root_layout.addLayout(left_layout)
        root_layout.addLayout(right_layout)

        self.line_edit = QLineEdit()
        self.line_edit.setPlaceholderText('Enter word')
        self.line_edit.setFont(font)
        tx_button = QPushButton()
        tx_button.setText('Send Morse!')
        tx_button.setFont(font)
        tx_button.clicked.connect(self.button_callback)
        left_layout.addWidget(self.line_edit, 0, 0)
        left_layout.addWidget(tx_button, 0, 1)

        self.morse_label = QLabel()
        self.morse_label.setFont(font)
        left_layout.addWidget(self.morse_label, 1, 0)
        self.morse_indicator = CircleIndicator()
        left_layout.addWidget(self.morse_indicator, 1, 1)

        self.set_light_signal.connect(self.set_light)

        map_path = str(
            Path(get_package_share_directory('gui')) / 'images' / 'morse_code.png'
        )
        map_pixmap = QPixmap(map_path)
        map_label = QLabel()
        map_label.setPixmap(
            map_pixmap.scaled(
                600,
                600,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )
        right_layout.addWidget(map_label)

        self.setLayout(root_layout)

    def button_callback(self) -> None:
        word = list(self.line_edit.text().upper())
        morse = [MORSE_DICT[letter] for letter in word]
        self.morse_label.setText(' '.join(morse))

        Thread(
            target=self.transmit_morse,
            args=[morse],
            daemon=True,
            name='morse_timer',
        ).start()

    def transmit_morse(self, morse: list[str]) -> None:
        # Preamble
        for _ in range(5):
            self.set_light_signal.emit(True)
            sleep(TIME_UNIT_S)
            self.set_light_signal.emit(False)
            sleep(TIME_UNIT_S)

        sleep(TIME_UNIT_S * 5)

        # Actual word
        for code in morse:
            for ditdah in code:
                self.set_light_signal.emit(True)
                sleep(TIME_UNIT_S * (1 if ditdah == DIT else 3))
                self.set_light_signal.emit(False)
                sleep(TIME_UNIT_S)
            sleep(TIME_UNIT_S * 3)

    @pyqtSlot(bool)
    def set_light(self, on: bool) -> None:
        self.morse_indicator.set_state(WidgetState.ON if on else WidgetState.OFF)

        manip_msg = Manip(manip_id='left', activated=on)
        self.manip_publisher.publish(manip_msg)