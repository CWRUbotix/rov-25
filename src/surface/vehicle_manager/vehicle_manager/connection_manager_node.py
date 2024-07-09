import time
from dataclasses import dataclass

import rclpy
from mavros_msgs.msg import State
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rov_msgs.msg import Heartbeat
from rov_msgs.msg import VehicleState as VehicleStateMsg

PI_TIMEOUT = 1  # Seconds


@dataclass
class VehicleState():
    pi_connected: bool = False
    pixhawk_connected: bool = False
    armed: bool = False


class VehicleManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("connection_manager_node", parameter_overrides=[])

        self.state_publisher = self.create_publisher(
            VehicleStateMsg, "vehicle_state_event", qos_profile_system_default
        )

        self.mavros_subscription = self.create_subscription(
            State,
            'mavros/state',
            self.mavros_callback,
            10
        )

        self.mavros_subscription = self.create_subscription(
            Heartbeat,
            'pi_heartbeat',
            self.heartbeat_callback,
            10
        )

        self.watchdog_timer = self.create_timer(1, self.watchdog_callback)
        self.last_heartbeat: float = 0  # Unix timestamp of the last mavros heartbeat from the pi

        self.last_subscriber_count = 0
        self.poll_subscribers_timer = self.create_timer(1, self.poll_subscribers)

        self.vehicle_state = VehicleState()

    def publish_state(self, state: VehicleState) -> None:
        self.state_publisher.publish(
            VehicleStateMsg(
                pi_connected=state.pi_connected,
                pixhawk_connected=state.pixhawk_connected,
                armed=state.armed
            )
        )

    def mavros_callback(self, msg: State) -> None:
        new_state = VehicleState(pi_connected=True,
                                 pixhawk_connected=msg.connected,
                                 armed=msg.armed)

        if new_state != self.vehicle_state:
            self.publish_state(new_state)

            if not self.vehicle_state.pi_connected:
                self.get_logger().info("Pi connected")

            if self.vehicle_state.pixhawk_connected and not new_state.pixhawk_connected:
                self.get_logger().warn("Pixhawk disconnected")
            elif not self.vehicle_state.pixhawk_connected and new_state.pixhawk_connected:
                self.get_logger().info("Pixhawk connected")

            if self.vehicle_state.armed and not new_state.armed:
                self.get_logger().info("Pixhawk disarmed")
            elif not self.vehicle_state.armed and new_state.armed:
                self.get_logger().info("Pixhawk armed")

            self.vehicle_state = new_state

    def heartbeat_callback(self, _: Heartbeat) -> None:
        self.last_heartbeat = time.time()

        if not self.vehicle_state.pi_connected:
            self.vehicle_state.pi_connected = True
            self.publish_state(self.vehicle_state)
            self.get_logger().info("Pi connected")

    def watchdog_callback(self) -> None:
        if self.vehicle_state.pi_connected and time.time() - self.last_heartbeat > PI_TIMEOUT:
            self.vehicle_state = VehicleState(
                pi_connected=False,
                pixhawk_connected=False,
                armed=False)
            self.publish_state(self.vehicle_state)
            self.get_logger().warn("Pi disconnected")

    def poll_subscribers(self) -> None:
        # Whenever a node subscribes to vehicle state updates, send the current state
        subscriber_count = self.state_publisher.get_subscription_count()
        if subscriber_count > self.last_subscriber_count:
            # TODO: debug messages seem broken
            # self.get_logger().debug("Subscriber connected; publishing state. "
            #                         f"Subscriber count={subscriber_count}")
            self.publish_state(self.vehicle_state)

        self.last_subscriber_count = subscriber_count


def main() -> None:
    rclpy.init()
    vehicle_manager = VehicleManagerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(vehicle_manager, executor=executor)


if __name__ == "__main__":
    main()
