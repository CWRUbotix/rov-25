from gui.gui_nodes.event_nodes.subscriber import GUIEventSubscriber
from gui.widgets.circle import CircleIndicator
from PyQt6.QtCore import pyqtSignal, pyqtSlot, QUrl
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget
from PyQt6.QtMultimedia import QSoundEffect

from rov_msgs.msg import Flooding

from ament_index_python.packages import get_package_share_directory
import os


# The 'Loop' enum has int values, not 'Loop', unbeknownst to mypy
Q_SOUND_EFFECT_LOOP_FOREVER: int = QSoundEffect.Loop.Infinite.value  # type: ignore


class FloodWarning(QWidget):

    signal = pyqtSignal(Flooding)

    def __init__(self) -> None:
        super().__init__()

        self.signal.connect(self.refresh)
        self.subscription = GUIEventSubscriber(Flooding, 'flooding', self.signal)
        # Create a latch variable
        self.warning_msg_latch = False
        # Create basic 2 vertical stacked boxes layout
        flood_layout = QVBoxLayout()
        # Create the label that tells us what this is

        header_layout = QHBoxLayout()
        label = QLabel('Flooding Status')
        font = QFont("Arial", 14)
        label.setFont(font)
        header_layout.addWidget(label)
        self.indicator_circle = CircleIndicator(radius=10)
        header_layout.addWidget(self.indicator_circle)

        flood_layout.addLayout(header_layout)

        self.indicator = QLabel('No Water present')
        self.indicator.setFont(font)
        flood_layout.addWidget(self.indicator)
        self.setLayout(flood_layout)

        alarm_sound_path = os.path.join(
            get_package_share_directory("gui"), "sounds", "alarm.wav"
        )
        self.alarm_sound = QSoundEffect()
        self.alarm_sound.setSource(QUrl.fromLocalFile(alarm_sound_path))
        self.alarm_sound.setLoopCount(Q_SOUND_EFFECT_LOOP_FOREVER)

    @pyqtSlot(Flooding)
    def refresh(self, msg: Flooding) -> None:
        if msg.flooding:
            self.indicator.setText('FLOODING')
            self.subscription.get_logger().error("Robot is actively flooding, do something!")
            self.warning_msg_latch = True
            self.indicator_circle.set_off()

            if not self.alarm_sound.isPlaying():
                self.alarm_sound.setLoopCount(Q_SOUND_EFFECT_LOOP_FOREVER)
                self.alarm_sound.play()
        else:
            self.indicator.setText('No Water present')
            self.indicator_circle.set_on()
            if self.warning_msg_latch:
                self.subscription.get_logger().warning("Robot flooding has reset itself.")
                self.warning_msg_latch = False
                self.alarm_sound.setLoopCount(0)
