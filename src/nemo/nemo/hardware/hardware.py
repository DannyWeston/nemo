import rclpy
from rclpy.node import Node

import socket
import fcntl
import struct

import math

from .thruster import Thruster

from geometry_msgs.msg import Twist

class Hardware(Node):
    def __init__(self):
        super().__init__('hardware')

        self.thrusters = { 
            "fl": Thruster(7),
            "fr": Thruster(11),
            "ul": Thruster(13),
            "ur": Thruster(15)
        }

        self.thruster_diff = 0.3

        self.max_thruster_speed = 0.5

        # Listen for thruster updates
        self.thruster_sub = self.create_subscription(Twist, "/nemo/props", self.thruster_callback, 1) # Queue size of 1

    def thruster_callback(self, msg):
        fl = msg.linear.x
        fr = msg.linear.x

        ul = msg.linear.z
        ur = msg.linear.z

        if (msg.angular.z < 0): fl -= msg.angular.z
        else: fr += msg.angular.z

        if (msg.angular.x < 0): ul -= msg.angular.x
        else: ur += msg.angular.x

        if self.max_thruster_speed * self.max_thruster_speed < self.square_magnitude(fl, fr):
            fl, fr = self.normalise(fl, fr)
            fl *= self.max_thruster_speed
            fr *= self.max_thruster_speed

        if self.max_thruster_speed * self.max_thruster_speed < self.square_magnitude(ul, ur):
            ul, ur = self.normalise(ul, ur)
            ul *= self.max_thruster_speed
            ur *= self.max_thruster_speed

        fl = (fl / self.max_thruster_speed) * 100.0
        fr = (fr / self.max_thruster_speed) * 100.0
        ul = (ul / self.max_thruster_speed) * 100.0
        ur = (ur / self.max_thruster_speed) * 100.0

        self.thrusters["fl"].set_speed(fl)
        self.thrusters["fr"].set_speed(fr)
        self.thrusters["ul"].set_speed(ul)
        self.thrusters["ur"].set_speed(ur)

    def magnitude(self, x, y):
        return math.sqrt(x * x + y * y)
    
    def square_magnitude(self, x, y):
        return x * x + y * y

    def normalise(self, x, y):
        dist = self.magnitude(x, y)
        return (x / dist, y / dist)

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
        rclpy.try_shutdown()
        node.destroy_node()
