from threading import Thread

from PyQt6.QtCore import pyqtBoundSignal
from rclpy.client import Client, SrvType, SrvTypeRequest
from rclpy.node import Node
from rclpy.publisher import MsgType
from rclpy.qos import QoSProfile, qos_profile_default
from rclpy.subscription import Subscription


class GUINode(Node):
    __instance: 'GUINode | None' = None

    def __new__(cls, _: str | None = None) -> 'GUINode':
        if cls.__instance is None:
            cls.__instance = super().__new__(cls)
        return cls.__instance

    def __init__(self, name: str | None = None) -> None:
        if name:
            super().__init__(name)

    # TODO: update sub to be generic in release after Jazzy
    def create_signal_subscription(
        self,
        msg_type: type[MsgType],
        topic: str,
        signal: pyqtBoundSignal,
        qos_profile: QoSProfile = qos_profile_default,
    ) -> Subscription:
        def wrapper(data: MsgType) -> None:
            return signal.emit(data)

        return super().create_subscription(msg_type, topic, wrapper, qos_profile)

    # # TODO: in the release after Iron can add back the Optional around timeout
    # # The fix internally is already out on Rolling
    # # Set to None for no timeout limits on service requests
    # # else set to float number of seconds to limit request spinning
    def create_client_multithreaded(
        self, srv_type: type[SrvType], srv_name: str, timeout: float = 10.0
    ) -> Client:
        cli = super().create_client(srv_type, srv_name)
        Thread(
            target=self.__connect_to_service,
            args=[cli, timeout],
            daemon=True,
            name=f'{srv_name}_connect_to_service',
        ).start()
        return cli

    def __connect_to_service(self, client: Client, timeout: float) -> None:
        """Connect this client to a server in a separate thread."""
        while not client.wait_for_service(timeout):
            super().get_logger().warning(
                'Service for GUI event client node on topic'
                f' {client.srv_name} unavailable, waiting again...'
            )

    @staticmethod
    def send_request_multithreaded(
        client: Client, request: SrvTypeRequest, signal: pyqtBoundSignal
    ) -> None:
        """Send request to server in separate thread."""

        def wrapper(request: SrvTypeRequest) -> None:
            signal.emit(client.call(request))

        Thread(
            target=wrapper,
            args=[request],
            daemon=True,
            name=f'{client.srv_name}_send_request',
        ).start()
