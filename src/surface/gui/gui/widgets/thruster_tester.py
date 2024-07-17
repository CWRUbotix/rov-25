import time
from threading import Thread

from mavros_msgs.srv import CommandLong
from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtGui import QIntValidator
from PyQt6.QtWidgets import QGridLayout, QLabel, QLineEdit, QPushButton, QVBoxLayout, QWidget

from gui.node_singleton import GUINode

class ThrusterTester(QWidget):
    """Widget to command the pixhawk to test the thrusters, and reassign thruster ports."""

    TEST_LENGTH: float = 2.0  # time between adjacent tests of individual thrusters
    TEST_THROTTLE: float = 0.50  # 50%
    MOTOR_COUNT = 8

    command_response_signal: pyqtSignal = pyqtSignal(CommandLong.Response)

    def __init__(self) -> None:
        super().__init__()

        self.cmd_client = GUINode().create_client_multithreaded(CommandLong, 'mavros/cmd/command')
        self.command_response_signal.connect(self.command_response_handler)

        layout = QVBoxLayout()
        self.setLayout(layout)

        heading = QLabel('Thruster Pin Configuration')

        pin_numbers_grid = QGridLayout()
        self.pin_input_widgets = []

        pin_number_validator = QIntValidator()
        pin_number_validator.setRange(0, self.MOTOR_COUNT - 1)

        for i in range(self.MOTOR_COUNT):
            pin_label = QLabel(str(i))
            pin_label.setMaximumWidth(20)
            pin_input = QLineEdit()
            pin_input.setValidator(pin_number_validator)
            pin_input.insert(str(i))
            pin_input.setMaximumWidth(20)
            self.pin_input_widgets.append(pin_input)

            on_first_column = i < self.MOTOR_COUNT / 2
            column = 1 if on_first_column else 4
            row = i if on_first_column else i - self.MOTOR_COUNT // 2
            pin_numbers_grid.addWidget(pin_label, row, column)
            pin_numbers_grid.addWidget(pin_input, row, column + 1)

        pin_numbers_grid.setColumnStretch(0, 1)
        pin_numbers_grid.setColumnStretch(3, 1)
        pin_numbers_grid.setColumnStretch(6, 1)

        pin_assignment_button = QPushButton()
        pin_assignment_button.setText('Send Pin Assignments')

        test_button = QPushButton()
        test_button.setText('Test Thrusters')
        test_button.clicked.connect(self.async_send_message)

        layout.addWidget(heading)
        layout.addLayout(pin_numbers_grid)
        layout.addWidget(pin_assignment_button)
        layout.addWidget(test_button)

    def test_motor_for_time(self, motor_index: int, throttle: float, duration: float) -> None:
        """
        Run a motor for an (approximate) length of time (blocking).

        Based on https://github.com/mavlink/qgroundcontrol/blob/f79b466/src/Vehicle/Vehicle.cc

        Parameters
        ----------
        motor_index : int
            A motor index, from 1 to 8
        throttle : float
            A float from -1 to 1, where -1 is full reverse and 1 is full forward
        duration : float
            Time in seconds to run the motor

        """
        throttle = max(min(throttle, 1.0), -1.0)  # Clamp the throttle between -1 and 1
        throttle_percent = 50 * throttle + 50  # Rescale to be from 0 to 100, with 50 at the center

        start_time = time.time()
        while time.time() - start_time < duration:
            GUINode.send_request_multithreaded(
                self.cmd_client,
                CommandLong.Request(
                    command=209,  # MAV_CMD_DO_MOTOR_TEST
                    param1=float(motor_index),  # Motor number
                    param2=0.0,  # MOTOR_TEST_THROTTLE_PERCENT
                    param3=float(throttle_percent),
                    param4=0.0,  # Time between tests
                    param5=0.0,  # Number of motors to test
                    param6=2.0,  # MOTOR_TEST_ORDER_BOARD
                ),
                self.command_response_signal,
            )
            time.sleep(0.05)

    def async_send_message(self) -> None:
        Thread(target=self.send_test_message, daemon=True, name='thruster_test_thread').start()

    def send_test_message(self) -> None:
        for motor_index in range(self.MOTOR_COUNT):
            GUINode().get_logger().info(f'Testing thruster {motor_index}')
            self.test_motor_for_time(motor_index, self.TEST_THROTTLE, self.TEST_LENGTH)
            self.test_motor_for_time(motor_index, 0.0, 0.5)

    def send_pin_assignments(self) -> None:
        # TODO: Send the pin assignments inputted by the user to the pixhawk as parameters
        # https://wiki.ros.org/mavros/Plugins#param
        # https://ardupilot.org/copter/docs/parameters.html#servo10-parameters
        pass

    @pyqtSlot(CommandLong.Response)
    def command_response_handler(self, res: CommandLong.Response) -> None:
        GUINode().get_logger().debug(f'Test response: {res.success}, {res.result}')
