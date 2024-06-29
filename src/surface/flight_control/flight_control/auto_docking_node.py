import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from rov_msgs.msg import PixhawkInstruction
from rov_msgs.srv import AutonomousFlight


class AutoDocker(Node):

    def __init__(self) -> None:
        super().__init__('auto_docker')

        self.control_server = self.create_service(
            AutonomousFlight, 'auto_control_toggle', self.task_control_callback)

        self.pixhawk_control = self.create_publisher(
            PixhawkInstruction,
            "pixhawk_control",
            QoSPresetProfiles.DEFAULT.value,
        )

        self.current_state = AutonomousFlight.Request.STOP

        # TODO: Add cam frame subscriber here to act as control loop for auto docking

    def task_control_callback(self, request: AutonomousFlight.Request,
                              response: AutonomousFlight.Response) -> AutonomousFlight.Response:
        self.current_state = request.state
        response.current_state = request.state
        return response


def main() -> None:
    rclpy.init()
    auto_docker = AutoDocker()
    executor = MultiThreadedExecutor()
    rclpy.spin(auto_docker, executor=executor)
