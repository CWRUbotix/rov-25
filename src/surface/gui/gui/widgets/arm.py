from mavros_msgs.srv import CommandBool
from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import QHBoxLayout, QWidget

from gui.gui_node import GUINode
from gui.styles.custom_styles import ButtonIndicator, WidgetState
from rov_msgs.msg import VehicleState


class Arm(QWidget):
    """Arm widget for sending Arm Commands."""

    ARM_REQUEST = CommandBool.Request(value=True)
    DISARM_REQUEST = CommandBool.Request(value=False)
    BUTTON_WIDTH = 120
    BUTTON_HEIGHT = 60
    BUTTON_STYLESHEET = 'QPushButton { font-size: 20px; }'

    command_response_signal = pyqtSignal(CommandBool.Response)
    vehicle_state_signal = pyqtSignal(VehicleState)

    def __init__(self) -> None:
        super().__init__()

        layout = QHBoxLayout()
        self.setLayout(layout)

        self.arm_button = ButtonIndicator(extra_styles=self.BUTTON_STYLESHEET)
        self.disarm_button = ButtonIndicator(extra_styles=self.BUTTON_STYLESHEET)

        self.arm_button.setText('Arm')
        self.disarm_button.setText('Disarm')

        self.arm_button.setMinimumWidth(self.BUTTON_WIDTH)
        self.disarm_button.setMinimumWidth(self.BUTTON_WIDTH)

        self.arm_button.setMinimumHeight(self.BUTTON_HEIGHT)
        self.disarm_button.setMinimumHeight(self.BUTTON_HEIGHT)

        self.arm_button.set_state(WidgetState.INACTIVE)
        self.disarm_button.set_state(WidgetState.INACTIVE)

        self.arm_button.clicked.connect(self.arm_clicked)
        self.disarm_button.clicked.connect(self.disarm_clicked)

        layout.addWidget(self.disarm_button)
        layout.addWidget(self.arm_button)

        self.command_response_signal.connect(self.arm_status)

        self.arm_client = GUINode().create_client_multithreaded(CommandBool, 'mavros/cmd/arming')

        GUINode().create_signal_subscription(
            VehicleState,
            'vehicle_state_event',
            self.vehicle_state_signal,
        )

        self.vehicle_state_signal.connect(self.vehicle_state_callback)

    def arm_clicked(self) -> None:
        GUINode().send_request_multithreaded(
            self.arm_client, self.ARM_REQUEST, self.command_response_signal
        )

    def disarm_clicked(self) -> None:
        GUINode().send_request_multithreaded(
            self.arm_client, self.DISARM_REQUEST, self.command_response_signal
        )

    @pyqtSlot(CommandBool.Response)
    def arm_status(self, res: CommandBool.Response) -> None:
        if not res:
            GUINode().get_logger().warning('Failed to arm or disarm.')

    @pyqtSlot(VehicleState)
    def vehicle_state_callback(self, msg: VehicleState) -> None:
        if msg.pixhawk_connected:
            if msg.armed:
                self.arm_button.set_state(WidgetState.ON)
                self.disarm_button.set_state(WidgetState.NONE)
            else:
                self.arm_button.set_state(WidgetState.NONE)
                self.disarm_button.set_state(WidgetState.OFF)
        else:
            self.arm_button.set_state(WidgetState.INACTIVE)
            self.disarm_button.set_state(WidgetState.INACTIVE)
