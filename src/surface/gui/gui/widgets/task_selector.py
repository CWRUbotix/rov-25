from gui.gui_nodes.event_nodes.client import GUIEventClient
from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import QGridLayout, QPushButton, QWidget
from gui.styles.custom_styles import ButtonIndicator
from rov_msgs.srv import AutonomousFlight

WIDTH = 200


class TaskSelector(QWidget):
    """QWidget that handles task selection with a dropdown."""

    scheduler_response_signal = pyqtSignal(AutonomousFlight.Response)

    def __init__(self) -> None:
        super().__init__()

        self.task_controller = GUIEventClient(AutonomousFlight, 'auto_control_toggle',
                                              self.scheduler_response_signal)

        layout = QGridLayout()
        self.setLayout(layout)

        # Create Start button
        self.start_btn = ButtonIndicator("Start Auto")
        self.start_btn.clicked.connect(
            lambda: self.task_controller.send_request_async(
                AutonomousFlight.Request(state=AutonomousFlight.Request.START)
            )
        )
        self.start_btn.setFixedHeight(75)
        self.start_btn.setFixedWidth(WIDTH)

        # Create Stop button
        stop_btn = QPushButton("Stop Auto")
        stop_btn.clicked.connect(
            lambda: self.task_controller.send_request_async(
                AutonomousFlight.Request(state=AutonomousFlight.Request.STOP)
            )
        )
        stop_btn.setFixedHeight(75)
        stop_btn.setFixedWidth(WIDTH)

        # Setup Grid
        layout.addWidget(self.start_btn, 2, 1)
        layout.addWidget(stop_btn, 3, 1)

        self.scheduler_response_signal.connect(self.handle_scheduler_response)

    @pyqtSlot(AutonomousFlight.Response)
    def handle_scheduler_response(self, res: AutonomousFlight.Response) -> None:
        """Handle scheduler response to request sent from gui_changed_task."""
        if res.current_state == AutonomousFlight.Request.START:
            self.start_btn.set_on()
        else:
            self.start_btn.remove_state()
