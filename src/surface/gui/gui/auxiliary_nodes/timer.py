from typing import TYPE_CHECKING

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rov_msgs.msg import MissionTimerTick
from rov_msgs.srv import MissionTimerSet

if TYPE_CHECKING:
    from rclpy.time import Time

PUBLISH_RATE = 10  # Hz
DEFAULT_DURATION = Duration(seconds=15 * 60)


class TimerNode(Node):
    """
    A ROS node which encapsulates a simple countdown timer.

    Publishes the time and whether it's counting down periodically and can be started, stopped,
    and reset via a service.
    """

    def __init__(self) -> None:
        """Initialize the publisher, service client, and internal variables."""
        super().__init__('timer_node', parameter_overrides=[])

        self.publisher = self.create_publisher(
            MissionTimerTick, 'mission_timer', qos_profile_system_default
        )

        self.set_time_service = self.create_service(
            MissionTimerSet, 'set_mission_timer', callback=self.set_time_callback
        )

        self.publisher_timer = self.create_timer(1 / PUBLISH_RATE, self.timer_callback)

        self.time_left = DEFAULT_DURATION
        self.is_running = False

        self.last_timestamp: Time | None = None

    def do_tick(self) -> None:
        """
        Update the time remaining on the timer.

        Reduce the timer's remaining duration by the duration that has elapsed on ROS's clock since
        the last time do_tick() was called.
        """
        timestamp = self.get_clock().now()

        if self.is_running and self.last_timestamp is not None:
            # TODO: Michael upstream typing oopsy
            # https://github.com/ros2/rclpy/pull/1312
            time_left = self.last_timestamp + self.time_left - timestamp
            if isinstance(time_left, Duration):
                self.time_left = time_left
            else:
                raise TypeError('self.time_left is not Duration')

            if self.time_left < Duration(seconds=0):
                self.is_running = False
                self.time_left = Duration(seconds=0)

        self.last_timestamp = timestamp

    def publish_tick_message(self) -> None:
        """Publish an update on the timer's states as a ROS message."""
        msg = MissionTimerTick()
        msg.is_running = self.is_running
        msg.time_left = self.time_left.to_msg()

        self.publisher.publish(msg)

    def timer_callback(self) -> None:
        """Advance the timer and publish an update; called periodically."""
        self.do_tick()
        self.publish_tick_message()

    def set_time_callback(
        self, request: MissionTimerSet.Request, response: MissionTimerSet.Response
    ) -> MissionTimerSet.Response:
        """
        Handle a request to start, stop, or reset the timer.

        Parameters
        ----------
        request : MissionTimerSet.Request
            The ROS request sent by the caller of the service; describes the desired action.
        response : MissionTimerSet.Response
            The default ROS response to be returned to the caller.

        Returns
        -------
        MissionTimerSet.Response
            The ROS response to be returned to the caller.
        """
        self.do_tick()

        if request.set_running:
            self.is_running = request.running

        if request.set_time:
            self.time_left = Duration.from_msg(request.time)
            self.last_timestamp = None

        self.publish_tick_message()

        response.success = True
        return response


def run_timer() -> None:
    """Run the timer node forever."""
    rclpy.init()
    timer_node = TimerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(timer_node, executor=executor)


if __name__ == '__main__':
    run_timer()
