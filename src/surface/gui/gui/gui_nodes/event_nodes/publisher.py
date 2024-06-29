from typing import Generic, TypeVar
import re

from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_system_default


MsgT = TypeVar('MsgT')


class GUIEventPublisher(Node, Generic[MsgT]):
    """Publisher for sending messages from the GUI."""

    def __init__(self, msg_type: type[MsgT], topic: str,
                 qos_profile: QoSProfile = qos_profile_system_default) -> None:
        # Name this node with a sanitized version of the topic
        name = f'publisher_{re.sub(r"[^a-zA-Z0-9_]", "_", topic)}'
        super().__init__(name, parameter_overrides=[])

        self.publisher = self.create_publisher(msg_type, topic, qos_profile)

    def publish(self, msg: MsgT) -> None:
        """Send a message with the provided parameters."""
        self.publisher.publish(msg)
