import atexit
import re
from threading import Thread

from PyQt6.QtCore import pyqtBoundSignal
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_system_default
from rclpy.subscription import MsgType


class GUIEventSubscriber(Node):
    """Multithreaded subscriber for receiving messages to the GUI."""

    def __init__(
        self,
        msg_type: MsgType,
        topic: str,
        signal: pyqtBoundSignal,
        qos_profile: QoSProfile = qos_profile_system_default,
    ) -> None:
        # Name this node with a sanitized version of the topic
        name: str = f'subscriber_{re.sub(r"[^a-zA-Z0-9_]", "_", topic)}'
        super().__init__(name, parameter_overrides=[])

        self.signal = signal

        self.subscription = self.create_subscription(
            msg_type, topic, lambda data: signal.emit(data), qos_profile
        )
        # Wrap in silly lambda becuase PyQ6 and ROS won't play nice

        custom_executor = SingleThreadedExecutor()
        custom_executor.add_node(self)
        Thread(target=custom_executor.spin, daemon=True, name=f'{name}_spin').start()
        atexit.register(custom_executor.shutdown)
