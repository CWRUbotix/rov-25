import atexit
import os
import subprocess
from subprocess import Popen
import re
from signal import SIGINT

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

WATCHDOG_RATE = 10
NAMESPACE = "surface"
MIN_FPS = 1.0


class Watchdog():
    def __init__(self, name: str, node: Node, args: list[str]) -> None:
        self.name = name
        self.node = node
        self.args = args
        self.process: Popen[bytes]

        self.start_process()

    def start_process(self) -> None:
        self.process = Popen(self.args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

        if self.process.stdout is None:
            raise RuntimeError("Child process has not stdout")

        os.set_blocking(self.process.stdout.fileno(), False)

    def read_stdout(self) -> bytes:
        if self.process.stdout is None:
            raise RuntimeError("Child process has not stdout")

        return self.process.stdout.readline()

    def poll(self) -> None:
        if self.process.poll() is not None:
            self.node.get_logger().warning(f"{self.name} has crashed, restarting...")
            self.start_process()
            return

        line = self.read_stdout()
        while line:
            match = re.search(r"rate \[Hz] in +([\d\.]+) out", line.decode().strip())
            if match:
                try:
                    rate = float(match.group(1))
                except ValueError:
                    continue

                if rate < MIN_FPS:
                    # If we're receiving less than 1 fps, assume the camera has disconnected
                    self.node.get_logger().warning(f"{self.name} frozen, killing...")
                    self.process.send_signal(SIGINT)
                    return

            line = self.read_stdout()

    def kill(self) -> None:
        self.process.kill()


class FlirWatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__("flir_watchdog_node")

        self.timer = self.create_timer(1 / WATCHDOG_RATE, self.timer_callback)

        self.get_logger().info("Starting camera drivers")

        self.front_watchdog = Watchdog(
            name="Front cam",
            node=self,
            args=["ros2", "launch", "rov_flir", "flir_launch.py",
                  "launch_bottom:=false", f"ns:={NAMESPACE}"]
        )
        self.bottom_watchdog = Watchdog(
            name="Bottom cam",
            node=self,
            args=["ros2", "launch", "rov_flir", "flir_launch.py",
                  "launch_front:=false", f"ns:={NAMESPACE}"]
        )

        atexit.register(self.front_watchdog.kill)
        atexit.register(self.bottom_watchdog.kill)

    def timer_callback(self) -> None:
        self.front_watchdog.poll()
        self.bottom_watchdog.poll()


def main() -> None:
    rclpy.init()
    flir_watchdog_node = FlirWatchdogNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(flir_watchdog_node, executor=executor)


if __name__ == "__main__":
    main()
