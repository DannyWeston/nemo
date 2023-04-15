# ROS Imports
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import Pose

# General Python imports

# Nemo Lib Imports
from ..lib.math import *

class Localiser(Node):
    def __init__(self):
        super().__init__('localiser')

        # Module for receiving odometry updates
        self.odom_position = None
        self.odom_orientation = None
        self.odom_sub = self.create_subscription(Odometry, "/nemo/odom", self.odom_callback, 1) # Queue size of 1

        # Module for receiving IMU updates
        # TODO: Add IMU support
        # Should use IMU heading as rotation instead of Odometry

        # Module for receiving pressure updates (Much mroe accurate position estimate than odometry)
        self.pressure = 0.0
        self.pressure_sub = self.create_subscription(FluidPressure, "/nemo/pressure", self.pressure_callback, 1) # Queue size of 1

        # Module for publishing pose updates
        self.pose_pub = self.create_publisher(Pose, "/nemo/pose", 10)

        # Setup updates
        self.declare_parameter('rate', 10)
        self.rate = self.get_parameter('rate').value
        self.create_timer(1.0 / self.rate, self.update)

    def update(self):
        if not self.odom_position: return # Can't publish position
        if not self.odom_orientation: return # Can't publish orientation

        position = list(self.odom_position)

        # if pressure sensor reading available, use it
        if self.pressure: position[2] = -(self.pressure - AIR_PRESSURE) / WATER_PRESSURE_PER_METRE # Convert to metres

        msg = Pose()
        msg.position.x = position[0]
        msg.position.y = position[1]
        msg.position.z = position[2]

        msg.orientation.x = self.odom_orientation.x
        msg.orientation.y = self.odom_orientation.y
        msg.orientation.z = self.odom_orientation.z
        msg.orientation.w = self.odom_orientation.w

        self.pose_pub.publish(msg)

    def odom_callback(self, msg):
        self.odom_position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)

        self.odom_orientation = msg.pose.pose.orientation

    def pressure_callback(self, msg):
        self.pressure = max(msg.fluid_pressure, AIR_PRESSURE)

def main(args=None):
    rclpy.init(args=args)

    node = Localiser()

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