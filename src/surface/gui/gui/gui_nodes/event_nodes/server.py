import atexit
import re
from threading import Thread
from typing import Callable

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.service import SrvType, SrvTypeRequest, SrvTypeResponse


class GUIEventServer(Node):
    """Multithreaded server for processing server requests to update GUI."""

    def __init__(
        self,
        srv_type: SrvType,
        topic: str,
        callback: Callable[[SrvTypeRequest, SrvTypeResponse], SrvTypeResponse],
    ):
        """
        Initialize this server with a CALLBACK for processing requests.

        Remember to use a signal to update the GUI!
        """
        # Name this node with a sanitized version of the topic
        name = f'server_{re.sub(r"[^a-zA-Z0-9_]", "_", topic)}'
        super().__init__(name, parameter_overrides=[])

        self.srv = self.create_service(srv_type, topic, callback)

        custom_executor = SingleThreadedExecutor()
        custom_executor.add_node(self)
        Thread(target=custom_executor.spin, daemon=True, name=f'{name}_spin').start()
        atexit.register(custom_executor.shutdown)
