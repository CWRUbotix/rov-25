import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rov_msgs.msg import Heartbeat

PUBLISH_RATE = 2  # Hz


class HeartbeatNode(Node):
    def __init__(self) -> None:
        super().__init__('heartbeat_node', parameter_overrides=[])

        self.publisher = self.create_publisher(
            Heartbeat, 'pi_heartbeat', qos_profile_system_default
        )

        self.timer = self.create_timer(1 / PUBLISH_RATE, self.timer_callback)

    def timer_callback(self) -> None:
        self.publisher.publish(Heartbeat())


def main() -> None:
    rclpy.init()
    heartbeat_node = HeartbeatNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(heartbeat_node, executor=executor)


if __name__ == '__main__':
    main()
