# ROS Imports
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from nav_msgs.msg import MapMetaData, OccupancyGrid as OccupancyGridMsg

from sensor_msgs.msg import Imu

from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import Pose, Point, Quaternion

from std_msgs.msg import Header

# General Python imports
import os
from datetime import datetime

# Nemo Lib Imports
from ..lib.math import *
from .occupancy_grid import OccupancyGrid

class Localiser(Node):
    def __init__(self):
        super().__init__('localiser')

        self.pose = Pose()
        self.start_pose = Pose()

        self.position = Point()
        self.orientation = Quaternion()

        self.pressure_pos = None

        self.save_map = True

        # Module for receiving odometry updates
        self.odom_sub = self.create_subscription(Odometry, "/nemo/odom", self.odom_callback, 1) # Queue size of 1

        # Module for receiving imu updates
        self.angular_vel = None
        self.linear_acceleration = None
        self.imu_sub = self.create_subscription(Imu, "/nemo/imu", self.imu_callback, 1) # Queue size of 1

        # Occupancy map with size 50x50 metres, resolution 3 cells per metre
        self.map = OccupancyGrid((50, 50), resolution=3, influence=0)

        # Module for receiving IMU updates
        # TODO: Add IMU support
        # Should use IMU heading as rotation instead of Odometry

        # Module for receiving pressure updates (Much mroe accurate position estimate than odometry)
        self.pressure_sub = self.create_subscription(FluidPressure, "/nemo/pressure", self.pressure_callback, 1) # Queue size of 1

        # Module for publishing pose updates
        self.pose_pub = self.create_publisher(Pose, "/nemo/pose", 10)

        # Send updates of the map
        self.map_pub = self.create_publisher(OccupancyGridMsg, "/nemo/map", 1) # Queue size of 1

        # Setup map output directory
        self.declare_parameter('map_dir', "/home/ros/maps")
        self.map_dir = self.get_parameter('map_dir').get_parameter_value().string_value
        self.map_file = os.path.join(self.map_dir, f'{datetime.now().strftime("%d%m%Y_%H%M%S")}.map')

        # Setup updates
        self.declare_parameter('rate', 5)
        self.rate = self.get_parameter('rate').value
        self.create_timer(1.0 / self.rate, self.update)

    def update(self):
        self.update_map()

        self.publish_pos()

        self.publish_map()

    def update_map(self):
        # Mark area in map as visited and flush to disk
        self.map.visit(self.position.x, self.position.y)

        if self.save_map: self.map.flush_to_disk(self.map_file)

    # Callbacks

    def odom_callback(self, msg):
        if not self.start_pose.position: self.start_pose.position = msg.pose.pose.position

        self.position = msg.pose.pose.position

    def pressure_callback(self, msg):
        water_pressure = max(msg.fluid_pressure, AIR_PRESSURE)
        self.pressure_pos = -(water_pressure - AIR_PRESSURE) / WATER_PRESSURE_PER_METRE

    def imu_callback(self, msg):
        if not self.start_pose.orientation: self.start_pose.orientation = msg.orientation

        self.orientation = msg.orientation
        self.angular_vel = msg.angular_velocity
        self.linear_accel = msg.linear_acceleration

    # Publishers

    def publish_map(self):
        if None in [self.start_pose.position, self.start_pose.orientation, self.map]: return

        metadata = MapMetaData()
        metadata.resolution = 1.0 / self.map.resolution
        metadata.width = self.map.size[0]
        metadata.height = self.map.size[1]
        metadata.origin = self.start_pose
        
        msg = OccupancyGridMsg()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = '/base_link'
        msg.info = metadata
        msg.data = self.map.grid.flatten().tolist() # TODO: Remove slow way of converting to 1D-list

        self.map_pub.publish(msg)

    def publish_pos(self):
        self.pose.position = self.position
        self.pose.orientation = self.orientation

        if self.pressure_pos: self.pose.position.z = self.pressure_pos

        self.pose_pub.publish(self.pose)

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
        rclpy.try_shutdown()
        node.destroy_node()