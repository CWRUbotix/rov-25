from threading import Thread

from PyQt6.QtCore import pyqtBoundSignal
from rclpy.client import Client, SrvType, SrvTypeRequest
from rclpy.node import Node
from rclpy.publisher import MsgType
from rclpy.qos import QoSProfile, qos_profile_default
from rclpy.subscription import Subscription


class GUINode(Node):
    """Singleton class that should be instantiated for each PyQt App.
    Then any Widget can import this class and call `GUINode()`
        to get access to that single instantiation.
    The first time `GUINode()` is called it creates a new node,
        and every following call to `GUINode()` retrieves that
        same node from memory.

    - Create publisher: `GUINode().create_publisher(...)`
    - Create subscription (calls callback function when message received):
    `GUINode().create_subscription(...)`
    - Create subscription (emits signal when message received):
    `GUINode().create_signal_subscription(...)`
    - Create client: `GUINode().create_client_multithreaded(...)`
    - Send client request (emits signal when request returns):
    `GUINode().send_request_multithreaded(...)`
    - Create service: `GUINode().create_service(...)`
    - Access logger: `GUINode().get_logger()`
    """

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
        """Create a subscription that emits messages to the given signal.
        See rclpy.node.Node.create_subcription for creating subscriptions
            based on callback methods instead of signals.

        Parameters
        ----------
        msg_type : type[MsgType]
            The message type of the subscription
        topic : str
            The topic name to subscribe to
        signal : pyqtBoundSignal
            The PyQt signal to emit messages to
        qos_profile : QoSProfile, optional
            The QoS profile for the subscription, by default qos_profile_default

        Returns
        -------
        Subscription
            The created Subscription
        """

        def wrapper(data: MsgType) -> None:
            return signal.emit(data)

        return super().create_subscription(msg_type, topic, wrapper, qos_profile)

    # Set to None for no timeout limits on service requests
    # else set to float number of seconds to limit request spinning
    def create_client_multithreaded(
        self, srv_type: type[SrvType], srv_name: str, timeout: float | None = 10.0
    ) -> Client:
        """Create a service client.
        On another thread, print warnings until it connects.

        Parameters
        ----------
        srv_type : type[SrvType]
            The service message type
        srv_name : str
            The topic name for this service
        timeout : float, optional
            The timeout between warning logs, by default 10.0

        Returns
        -------
        Client
            The created Client
        """
        cli = super().create_client(srv_type, srv_name)
        Thread(
            target=self.__connect_to_service,
            args=[cli, timeout],
            daemon=True,
            name=f'{srv_name}_connect_to_service',
        ).start()
        return cli

    def __connect_to_service(self, client: Client, timeout: float) -> None:
        """Print warnings until the given client connects (blocking).

        Parameters
        ----------
        client : Client
            The client to wait for
        timeout : float
            The timeout between warning logs
        """
        while not client.wait_for_service(timeout):
            super().get_logger().warning(
                'Service for GUI event client node on topic'
                f' {client.srv_name} unavailable, waiting again...'
            )

    def send_request_multithreaded(
        self, client: Client, request: SrvTypeRequest, signal: pyqtBoundSignal
    ) -> None:
        """Send a request from the given client on a separate thread.
        Emit the result to the given signal.

        Parameters
        ----------
        client : Client
            The client to make the request from
        request : SrvTypeRequest
            The request to send
        signal : pyqtBoundSignal
            The signal to emit the result to
        """

        def wrapper(request: SrvTypeRequest) -> None:
            signal.emit(client.call(request))

        Thread(
            target=wrapper,
            args=[request],
            daemon=True,
            name=f'{client.srv_name}_send_request',
        ).start()
