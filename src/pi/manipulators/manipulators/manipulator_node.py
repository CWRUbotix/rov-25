import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default
from rov_msgs.msg import Manip
from tca9555 import TCA9555

ALL_BITS = (0, 1, 2, 3, 4, 5)


class Manipulator(Node):

    def __init__(self) -> None:
        super().__init__('manipulator',
                         parameter_overrides=[])

        self.subscription = self.create_subscription(
            Manip,
            'manipulator_control',
            self.manip_callback,
            qos_profile_system_default
        )

        self.declare_parameters(
            namespace="",
            parameters=[
                ("left", Parameter.Type.INTEGER),
                ("right", Parameter.Type.INTEGER)
            ])

        # Initialize with standard I2C-bus address of TCA9555 a.k.a 0x20
        self.i2c = TCA9555()  # can put in the address as a param in hexadecimal
        self.get_logger().info(str(self.i2c.format_config()))

        # Set pins 0 through 5 as output
        self.i2c.set_direction(0, bits=ALL_BITS)
        self.i2c.unset_bits(bits=ALL_BITS)

    def manip_callback(self, message: Manip) -> None:
        manip_id = message.manip_id
        activated = message.activated

        if manip_id != "valve":
            pin = self.get_parameter(manip_id).get_parameter_value().integer_value

            if activated:
                self.i2c.set_bits(bits=(pin))
            else:
                self.i2c.unset_bits(bits=(pin))


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
