import rclpy
from rclpy.node import Node

import socket
import fcntl
import struct

class Hardware(Node):
    def __init__(self):
        super().__init__('hardware')

        self.get_logger().info(self.get_ip_address('eth0'))

    def get_ip_address(self, ifname):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', ifname[:15])
        )[20:24])

def main(args=None):
    rclpy.init(args=args)

    node = Hardware()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.shutdown()
        rclpy.try_shutdown()
        node.destroy_node()
