from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtGui import QTextCursor
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QTextEdit, QVBoxLayout, QWidget
from pyqtgraph import PlotWidget
from rclpy.qos import qos_profile_default
from rov_msgs.msg import FloatCommand, FloatData, FloatSerial, FloatSingle

from gui.node_singleton import GUINode
from rclpy.qos import qos_profile_default


class FloatComm(QWidget):
    """FloatComm widget for sending Float Communication Commands."""

    handle_data_signal = pyqtSignal(FloatData)
    handle_serial_signal = pyqtSignal(FloatSerial)
    handle_data_single_signal = pyqtSignal(FloatSingle)

    def __init__(self) -> None:
        super().__init__()

        layout = QHBoxLayout()
        self.setLayout(layout)

        self.handle_data_signal.connect(self.handle_data)
        self.handle_serial_signal.connect(self.handle_serial)
        self.handle_data_single_signal.connect(self.handle_single)
        GUINode().create_signal_subscription(FloatData, 'transceiver_data', self.handle_data_signal)
        GUINode().create_signal_subscription(FloatSerial, 'float_serial', self.handle_serial_signal)
        GUINode().create_signal_subscription(
            FloatSingle, 'transceiver_single', self.handle_data_single_signal
        )

        info_layout = QVBoxLayout()

        single_layout = QHBoxLayout()

        self.team_number = QLabel('Waiting for Team #')
        self.time = QLabel('Waiting for Time')
        self.pressure = QLabel('Waiting for Pressure')
        self.average_pressure = QLabel('Avg Pressure: 0/5')

        single_layout.addWidget(self.team_number)
        single_layout.addWidget(self.time)
        single_layout.addWidget(self.pressure)
        single_layout.addWidget(self.average_pressure)

        self.single_time = QLabel('Waiting for Time')
        self.single_pressure = QLabel('Waiting for Pressure')
        self.profile_number = QLabel('Waiting for profile #')
        self.profile_half = QLabel('Waiting for profile half')

        info_layout.addWidget(self.profile_number)
        info_layout.addWidget(self.profile_half)

        info_and_buttons = QHBoxLayout()
        info_and_buttons.addLayout(info_layout)
        info_and_buttons.addLayout(self.make_button_layout())

        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)

        font = self.console.font()
        font.setFamily('Courier')
        font.setPointSize(11)
        self.console.setFont(font)

        self.plots = [PlotWidget(), PlotWidget()]

        left_side_layout = QVBoxLayout()
        left_side_layout.addLayout(info_and_buttons)
        left_side_layout.addLayout(single_layout)
        left_side_layout.addWidget(self.console)

        layout.addLayout(left_side_layout)
        for plot in self.plots:
            layout.addWidget(plot)

        self.time_data: list[float] = []
        self.depth_data: list[float] = []
        self.received_first_half = False
        self.received_second_half = False
        self.completed_profile_one = False

        self.counter = 0

    def make_button_layout(self) -> QHBoxLayout:
        button_layout = QHBoxLayout()

        command_pub = GUINode().create_publisher(FloatCommand, 'float_command', qos_profile_default)

        submerge_button = QPushButton('Submerge')
        pump_button = QPushButton('Pump')
        suck_button = QPushButton('Suck')
        return_button = QPushButton('Return')
        stop_button = QPushButton('Stop')

        submerge_button.clicked.connect(
            lambda: command_pub.publish(FloatCommand(command=FloatCommand.SUBMERGE))
        )
        pump_button.clicked.connect(
            lambda: command_pub.publish(FloatCommand(command=FloatCommand.PUMP))
        )
        suck_button.clicked.connect(
            lambda: command_pub.publish(FloatCommand(command=FloatCommand.SUCK))
        )
        return_button.clicked.connect(
            lambda: command_pub.publish(FloatCommand(command=FloatCommand.RETURN))
        )
        stop_button.clicked.connect(
            lambda: command_pub.publish(FloatCommand(command=FloatCommand.STOP))
        )

        button_layout.addWidget(submerge_button)
        button_layout.addWidget(pump_button)
        button_layout.addWidget(suck_button)
        button_layout.addWidget(return_button)
        button_layout.addWidget(stop_button)

        return button_layout

    @pyqtSlot(FloatData)
    def handle_data(self, msg: FloatData) -> None:
        """
        Set the widget label text to the message in the FloatCommand.

        Parameters
        ----------
        msg : FloatData
            the data from the float
        """
        self.team_number.setText(f'Team #: {msg.team_number}')
        self.profile_number.setText(f'Profile #: {msg.profile_number}')
        self.profile_half.setText(f'Profile half: {msg.profile_half}')

        time_data = list(msg.time_data)
        depth_data = list(msg.depth_data)

        if (
            msg.profile_number == 0
            and self.completed_profile_one
            or msg.profile_number not in (0, 1)
        ):
            return

        if msg.profile_half == 0 and not self.received_first_half:
            self.time_data = time_data + self.time_data
            self.depth_data = depth_data + self.depth_data
            self.received_first_half = True
        elif msg.profile_half == 1 and not self.received_second_half:
            self.time_data = self.time_data + time_data
            self.depth_data = self.depth_data + depth_data
            self.received_second_half = True

        if self.received_first_half and self.received_second_half:
            self.plots[msg.profile_number].plot(self.time_data, self.depth_data)
            self.time_data = []
            self.depth_data = []
            self.received_first_half = False
            self.received_second_half = False
            self.completed_profile_one = True

    @pyqtSlot(FloatSerial)
    def handle_serial(self, msg: FloatSerial) -> None:
        """
        Set the widget label text to the message in the FloatCommand.

        Parameters
        ----------
        msg : FloatSerial
            the serial from the float
        """
        self.console.moveCursor(QTextCursor.MoveOperation.End)
        self.console.insertPlainText(f'{msg.serial}\n')

    @pyqtSlot(FloatSingle)
    def handle_single(self, msg: FloatSingle) -> None:
        self.counter += 1

        self.team_number.setText(f'Team #: {msg.team_number}')
        self.time.setText(f'Time: {msg.time_ms} (ms)')
        # Magic mbar -> Kpa
        pressure = round(msg.pressure / 10, 4)
        avg_pressure = round(msg.average_pressure / 10, 4)

        self.pressure.setText(f'Pressure: {pressure} (kPa)')
        if msg.average_pressure != 0.0:
            self.average_pressure.setText(f'Avg Pressure: {avg_pressure} (kPa)')
        else:
            self.average_pressure.setText(f'Avg Pressure: {self.counter}/5')
