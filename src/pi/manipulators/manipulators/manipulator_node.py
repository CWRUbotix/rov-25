import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default
from smbus2 import SMBus, i2c_msg

from rov_msgs.msg import Manip

I2C_BUS = 6
ADRRESS = 0x20
CMD_BYTE = 0x06


class Manipulator(Node):
    def __init__(self) -> None:
        super().__init__('manipulator', parameter_overrides=[])

        self.subscription = self.create_subscription(
            Manip, 'manipulator_control', self.manip_callback, qos_profile_system_default
        )

        self.declare_parameters(
            namespace='',
            parameters=[('left', Parameter.Type.INTEGER), ('right', Parameter.Type.INTEGER)],
        )

        self.i2c = SMBus(I2C_BUS)
        self.state = 0

    def manip_callback(self, message: Manip) -> None:
        manip_id = message.manip_id
        activated = message.activated

        if manip_id != 'valve':
            pin = self.get_parameter(manip_id).get_parameter_value().integer_value

            self.state &= ~(2**pin)  # Unset the pin with a bitmask
            if activated:
                self.state |= 2**pin  # Set the pin

            msg = i2c_msg.write(ADRRESS, [CMD_BYTE, self.state])
            self.i2c.i2c_rdwr(msg)


def main() -> None:
    rclpy.init()

    subscriber = Manipulator()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
