from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QGridLayout, QLabel, QPushButton, QWidget
from rclpy.duration import Duration
from rov_msgs.msg import MissionTimerTick
from rov_msgs.srv import MissionTimerSet

from gui.widgets.node_singleton import GUINode

RESET_SECONDS = 15 * 60  # The number of seconds to set the timer to when reset is clicked


class TimerDisplay(QLabel):
    """Widget which displays in real time the time left on a ROS countdown timer."""

    mission_timer_signal = pyqtSignal(MissionTimerTick)

    def __init__(self) -> None:
        """Initialize a ROS subscriber and set up a Qt label."""
        super().__init__()

        GUINode().create_signal_subscription(
            MissionTimerTick,
            'mission_timer',
            self.mission_timer_signal,
        )
        self.mission_timer_signal.connect(self.mission_timer_callback)

        self.running = False

        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setFont(QFont('Arial', 36))
        self.setText('--:--')

    def update_label(self, seconds_left: int) -> None:
        """
        Update the text on the Qt label to show the number of seconds remaining on the timer.

        Parameters
        ----------
        seconds_left : int
            The integer number of seconds to be displayed.
        """
        minutes = seconds_left // 60
        seconds = seconds_left % 60
        self.setText(f'{minutes:02d}:{seconds:02d}')

    @pyqtSlot(MissionTimerTick)
    def mission_timer_callback(self, msg: MissionTimerTick) -> None:
        """
        Handle an update message from the ROS timer node.

        Parameters
        ----------
        msg : MissionTimerTick
            The ROS message object containing the timer's timer left and whether it's running.
        """
        self.running = msg.is_running

        # Round seconds up, so the timer only displays 0 when the time remaining is actually 0
        seconds_left = msg.time_left.sec + (msg.time_left.nanosec > 0)
        self.update_label(seconds_left)


class InteractiveTimer(QWidget):
    """An interactive Qt interface for a ROS timer node."""

    set_timer_response_signal: pyqtSignal = pyqtSignal(MissionTimerSet.Response)

    def __init__(self) -> None:
        """
        Initialize the ROS widget.

        Create a ROS client to send requests to the timer node, and initialize the
        Qt widgets and layout
        """
        super().__init__()

        self.set_timer_client = GUINode().create_client_multithreaded(MissionTimerSet, 'set_mission_timer')
        self.set_timer_response_signal.connect(self.set_time_response_callback)

        self.timer = TimerDisplay()
        self.timer.mission_timer_signal.connect(self.mission_timer_callback)

        self.toggle_btn = QPushButton('Start')
        self.reset_btn = QPushButton('Reset')

        self.toggle_btn.setCheckable(True)

        self.toggle_btn.clicked.connect(self.toggle_timer)
        self.reset_btn.clicked.connect(self.reset_timer)

        layout = QGridLayout()
        layout.setContentsMargins(0, 0, 0, 0)

        layout.addWidget(self.timer, 0, 0, 1, 2)
        layout.addWidget(self.toggle_btn, 1, 0)
        layout.addWidget(self.reset_btn, 1, 1)

        self.setLayout(layout)

    def mission_timer_callback(self, msg: MissionTimerTick) -> None:
        """
        Handle an update message from the ROS timer node.

        Parameters
        ----------
        msg : MissionTimerTick
            The ROS message object containing the timer's timer left and whether it's running.
        """
        self.toggle_btn.setChecked(msg.is_running)
        if msg.is_running:
            self.toggle_btn.setText('Pause')
        else:
            self.toggle_btn.setText('Start')

    def toggle_timer(self) -> None:
        """If the ROS timer is running, pause it. If it's paused, resume it."""
        GUINode.send_request_multithreaded(
            self.set_timer_client,
            MissionTimerSet.Request(set_running=True, running=not self.timer.running),
            self.set_timer_response_signal,
        )

    def reset_timer(self) -> None:
        """Stop the timer and reset its remaining duration to the default value."""
        GUINode.send_request_multithreaded(
            self.set_timer_client,
            MissionTimerSet.Request(
                set_time=True,
                time=Duration(seconds=RESET_SECONDS).to_msg(),
                set_running=True,
                running=False,
            ),
            self.set_timer_response_signal,
        )

    @pyqtSlot(MissionTimerSet.Response)
    def set_time_response_callback(self, res: MissionTimerSet.Response) -> None:
        """
        Handle a response to a service call made by this object.

        Parameters
        ----------
        res : MissionTimerSet.Response
            The ROS response sent by the timer node.
        """
        if not res.success:
            GUINode().get_logger().error('Failed to set timer')
