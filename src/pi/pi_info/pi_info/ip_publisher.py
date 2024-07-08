import fcntl
import socket
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rov_msgs.msg import IPAddress


class IPPublisher(Node):
    def __init__(self) -> None:
        """Create IP Publisher node."""
        super().__init__('ip_publisher')
        self.publisher = self.create_publisher(
            IPAddress, 'ip_address', qos_profile_system_default
        )
        timer_period = 0.5  # seconds
        self.create_timer(timer_period, self.timer_callback)
        self.failed_ethernet = False
        self.failed_wireless = False

    def timer_callback(self) -> None:
        """On timer publishes the ip address of the computer."""
        msg = IPAddress()

        try:
            msg.ethernet_address = get_ip_address('eth0')
            self.failed_ethernet = False
        except OSError:
            if not self.failed_ethernet:
                self.get_logger().warn('No ethernet IP address found.')
            self.failed_ethernet = True

        try:
            msg.wireless_address = get_ip_address('wlan0')
            self.failed_wireless = False
        except OSError:
            if not self.failed_wireless:
                self.get_logger().warn('No wireless IP address found.')
            self.failed_wireless = True

        self.publisher.publish(msg)


# https://stackoverflow.com/questions/24196932/how-can-i-get-the-ip-address-from-a-nic-network-interface-controller-in-python
def get_ip_address(ifname: str = 'eth0') -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(
        fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', ifname[:15].encode()),
        )[20:24]
    )


def main() -> None:
    rclpy.init()
    minimal_publisher = IPPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
