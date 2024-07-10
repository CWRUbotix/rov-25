import re
from threading import Thread

import rclpy
from PyQt6.QtCore import pyqtBoundSignal
from rclpy.client import SrvType, SrvTypeRequest
from rclpy.node import Node


class GUIEventClient(Node):
    """Multithreaded client for sending service requests from the GUI."""

    # TODO: in the release after Iron can add back the Optional around timeout
    # The fix internally is already out on Rolling
    # Set to None for no timeout limits on service requests
    # else set to float number of seconds to limit request spinning
    def __init__(
        self,
        srv_type: SrvType,
        topic: str,
        signal: pyqtBoundSignal,
        timeout: float = 3.0,
        expected_namespace: str = '/surface/gui',
    ) -> None:
        # Name this node with a sanitized version of the topic
        self.name = f'client_{re.sub(r"[^a-zA-Z0-9_]", "_", topic)}'
        super().__init__(self.name, parameter_overrides=[])

        self.srv_type = srv_type
        self.topic = topic
        self.signal = signal
        self.timeout = timeout
        self.expected_namespace = expected_namespace

        self.cli = self.create_client(srv_type, topic)
        Thread(
            target=self.__connect_to_service, daemon=True, name=f'{self.name}_connect_to_service'
        ).start()

    def __connect_to_service(self) -> None:
        """Connect this client to a server in a separate thread."""
        while not self.cli.wait_for_service(timeout_sec=self.timeout):
            self.get_logger().info(
                'Service for GUI event client node on topic'
                f' {self.expected_namespace}/{self.topic} unavailable, waiting again...'
            )

    def send_request_async(self, request: SrvTypeRequest) -> None:
        """Send request to server in separate thread."""
        Thread(
            target=self.__send_request_with_signal,
            kwargs={'request': request},
            daemon=True,
            name=f'{self.name}_send_request',
        ).start()

    def __send_request_with_signal(self, request: SrvTypeRequest) -> None:
        """Send synchronous request to server and emit signal."""
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)

        try:
            result = future.result()
            if result is not None:
                self.signal.emit(result)
            else:
                self.get_logger().error('Request received no response.')
        except TypeError as exception:
            self.get_logger().error('Request received incorrectly typed response.')
            self.get_logger().error(str(exception))
