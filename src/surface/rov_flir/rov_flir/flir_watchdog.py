import atexit
import os
import re
import subprocess
from signal import SIGINT
from subprocess import Popen, TimeoutExpired

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rov_msgs.srv import CameraManage

WATCHDOG_RATE = 10
NAMESPACE = 'surface'
MIN_FPS = 1.0
KILL_TIMEOUT_S = 5


class Watchdog:
    def __init__(
        self, name: str, node: Node, args: list[str], *, should_be_alive: bool = True
    ) -> None:
        self.name = name
        self.node = node
        self.args = args
        self.process: Popen[bytes]

        self.should_be_alive = should_be_alive

        if self.should_be_alive:
            self._start_process()

    def _start_process(self) -> None:
        """
        Start the process and set should_be_alive.

        Raises
        ------
        RuntimeError
            if the started process does not have stdout
        """
        self.process = Popen(self.args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

        self.should_be_alive = True

        if self.process.stdout is None:
            raise RuntimeError('Child process has no stdout')

        os.set_blocking(self.process.stdout.fileno(), False)

    def start_process(self) -> bool:
        """
        Start the process if it isn't running.

        Returns
        -------
        bool
            True if the process is now alive; False otherwise
        """
        if self.is_alive():
            return True

        try:
            self._start_process()
        except RuntimeError:
            return False

        return True

    def kill_process(self) -> bool:
        """
        Kill the process and unset should_be_alive.

        Returns
        -------
        bool
            True if the process is now dead; False otherwise
        """
        self.should_be_alive = False
        # Alternative killing (doesn't work): self.process.kill()
        self.process.send_signal(SIGINT)
        self.node.get_logger().info(f'Killing FLIR ${self.name} cam')
        try:
            self.process.wait(timeout=KILL_TIMEOUT_S)
            return not self.is_alive()
        except TimeoutExpired:
            return False

    def _read_stdout(self) -> bytes:
        """
        Read a line of stdout from the process.

        Returns
        -------
        bytes
            the bytes read

        Raises
        ------
        RuntimeError
            if the process does not have stdout
        """
        if self.process.stdout is None:
            raise RuntimeError('Child process has no stdout')

        return self.process.stdout.readline()

    def poll(self) -> None:
        """Actively keep the process alive if should_be_alive is set."""
        if self.should_be_alive:
            self.keep_alive()

    def is_alive(self) -> bool:
        """
        Check if the process is alive.

        Returns
        -------
        bool
            True if the process is alive; False otherwise
        """
        return self.process.poll() is None

    def keep_alive(self) -> None:
        """Restart the process if it crashed."""
        if not self.is_alive():
            self.node.get_logger().warning(f'{self.name} has crashed, restarting...')
            self._start_process()
            return

        line = self._read_stdout()
        while line:
            match = re.search(r'rate \[Hz] in +([\d\.]+) out', line.decode().strip())
            if match:
                try:
                    rate = float(match.group(1))
                except ValueError:
                    continue

                if rate < MIN_FPS:
                    # If we're receiving less than 1 fps, assume the camera has disconnected
                    self.node.get_logger().warning(f'{self.name} frozen, killing...')
                    self.process.send_signal(SIGINT)
                    return

            line = self._read_stdout()


class FlirWatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__('flir_watchdog_node')

        self.timer = self.create_timer(1 / WATCHDOG_RATE, self.timer_callback)

        self.get_logger().info('Starting camera drivers')

        self.front_watchdog = Watchdog(
            name='Front cam',
            node=self,
            args=[
                'ros2',
                'launch',
                'rov_flir',
                'flir_launch.py',
                'launch_bottom:=false',
                f'ns:={NAMESPACE}',
            ],
        )
        self.bottom_watchdog = Watchdog(
            name='Bottom cam',
            node=self,
            args=[
                'ros2',
                'launch',
                'rov_flir',
                'flir_launch.py',
                'launch_front:=false',
                f'ns:={NAMESPACE}',
            ],
        )

        self.cam_manage_service = self.create_service(
            CameraManage, 'manage_flir', self.manage_cams_callback
        )

        atexit.register(self.front_watchdog.kill_process)
        atexit.register(self.bottom_watchdog.kill_process)

    def manage_cams_callback(
        self, request: CameraManage.Request, response: CameraManage.Response
    ) -> CameraManage.Response:
        """
        Handle CameraManage message by changing camera process states.

        Parameters
        ----------
        request : CameraManage.Request
            the incoming service request
        response : CameraManage.Response
            a template for the outgoing service response

        Returns
        -------
        CameraManage.Response
            the completed service response
        """
        if request.cam == CameraManage.Request.DOWN:
            self.get_logger().info(f'Received down cam {"on" if request.on else "off"} request')
            target_watchdog = self.bottom_watchdog
        elif request.cam == CameraManage.Request.FRONT:
            self.get_logger().info(f'Received front cam {"on" if request.on else "off"} request')
            target_watchdog = self.front_watchdog
        else:
            response.success = False
            return response

        if request.on:
            response.success = target_watchdog.start_process()
        else:
            response.success = target_watchdog.kill_process()

        return response

    def timer_callback(self) -> None:
        self.front_watchdog.poll()
        self.bottom_watchdog.poll()


def main() -> None:
    rclpy.init()
    flir_watchdog_node = FlirWatchdogNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(flir_watchdog_node, executor=executor)


if __name__ == '__main__':
    main()
