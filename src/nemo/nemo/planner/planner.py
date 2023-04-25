# ROS Imports
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt8
from std_msgs.msg import Float32

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

# General Python imports
from enum import Enum

import time
import random

# Nemo Lib Imports

from nemo_interfaces.msg import TrackerMsg

from .autodepth import AutoDepth

from ..lib.math import quat_to_euler

from ..lib.pid import PID

class Planner(Node):
    def __init__(self):
        super().__init__('planner')

        params = self.get_parameters()

        self.p_state = PlannerState.Searching # Default planner state is searching
        self.c_state = ControlState.Paused # Default control state is paused

        # Module for tracking fish
        self.found_id = None
        self.found_box = None
        self.found_width = None
        self.fish_pid = PID(kp=5, ki=1, kd=3, min=-1.0, max=1.0)
        self.camera_sub = self.create_subscription(TrackerMsg, "/nemo/tracker", self.tracker_callback, 1) # Queue size of 1

        # Module for maintaining a set depth
        self.depth = AutoDepth(params['autodepth.kp'], params['autodepth.ki'], params['autodepth.kd'], params['autodepth.min_depth'], params['autodepth.max_depth'], params['autodepth.tolerance'])
        self.depth_control_sub = self.create_subscription(Float32, "/nemo/depth", self.depth_control_callback, 1) # Queue size of 1

        # Module for retrieving Nemo's location
        self.position = None
        self.orientation = None
        self.quat_orientation = None # Orientation as Quaternion
        self.pose_sub = self.create_subscription(Pose, "/nemo/pose", self.pose_callback, 1) # Queue size of 1

        self.random_start = None

        # Module for sending movement updates to the hardware
        self.linear_vel = [0.0, 0.0, 0.0]
        self.angular_vel = [0.0, 0.0, 0.0]
        self.mover_pub = self.create_publisher(Twist, "/nemo/props", 10)

        # Receive updates for planner state updates
        self.planner_sub = self.create_subscription(UInt8, "/nemo/planner", self.planner_callback, 1) # Queue size of 1

        # Receive updates for control state updates
        self.control_sub = self.create_subscription(UInt8, "/nemo/control", self.control_callback, 1) # Queue size of 1

        # Setup updates
        self.declare_parameter('rate', 3)
        self.rate = self.get_parameter('rate').value
        self.create_timer(1.0 / self.rate, self.update)

    # Update functions
    
    def update(self):
        match self.c_state:
            case ControlState.ManualToAuto: # Transitioning to automatic control
                self.zero_vel()
                self.publish_vel()
                self.depth.reset()
                self.get_logger().info("Changing to autonomous control")
                self.c_state = ControlState.Auto
            
            case ControlState.AutoToManual: # Transitioning to automatic control
                self.zero_vel()
                self.publish_vel()
                self.get_logger().info("Changing to manual control")
                self.c_state = ControlState.Manual

            case ControlState.Auto:
                self.auto_update()

            case ControlState.Manual:
                # Do nothing
                pass

            case ControlState.Paused: # TODO: Ensure EVERYTHING is stopped
                self.zero_vel()
                self.publish_vel()

            case _:
                self.zero_vel()
                self.publish_vel()
                self.c_state = ControlState.Paused
                self.get_logger().info("Impossible control state reached, defaulting to paused")

    def auto_update(self):
        # Autononmous mode

        # Hazards get priority
        self.check_hazards()

        # Maintain depth
        if (self.position):
            self.linear_vel[2] = self.depth.update(self.position[2])

        match self.p_state:
            case PlannerState.Avoiding:
                self.avoid_hazard()

            case PlannerState.Searching:
                self.search_goal()

            case PlannerState.Following:
                self.follow_goal()

        self.publish_vel()

    def check_hazards(self):
        hazard = False
        if hazard: self.p_state = PlannerState.Avoiding

    def avoid_hazard(self):
        self.get_logger().info("Attempting to avoid a hazard...")

    def follow_goal(self):
        self.get_logger().info("Following identified goal...")

        # Drive towards goal at 0.2 m/s
        self.linear_vel[0] = 0.2

        # Calculate distance between bounding box center and center of screen
        dist = (self.found_box[2] - self.found_box[0] - self.found_width) / 2.0  # Camera width

        self.angular_vel[2] = self.fish_pid.update(dist) # Update yaw with resultant of PID

    def search_goal(self):
        if self.random_start:
            now = time.time()
            elapsed = now - self.random_start
            if 5 < now: # If 5 seconds has passed since the start of the random walk  
                self.random_start = None
                return
        else:
            self.linear_vel[0] = 0.3
            self.angular_vel[2] = random.uniform(-0.3, 0.3)

            self.random_start = time.time()

    # Functions

    def zero_vel(self):
        self.linear_vel = [0.0, 0.0, 0.0]
        self.angular_vel = [0.0, 0.0, 0.0]

    # Callbacks

    def planner_callback(self, msg):
        self.p_state = PlannerState(int(msg.data))

    def control_callback(self, msg):
        self.c_state = ControlState(int(msg.data))

    def tracker_callback(self, msg):
        self.found_id = msg.id
        self.found_box = [msg.x1, msg.y1, msg.x2, msg.y2]
        self.found_width = msg.width
        self.get_logger().info(f'Fish {self.found_id} identified')

    def pose_callback(self, msg):
        self.position = (msg.position.x, msg.position.y, msg.position.z)

        self.quat_orientation = msg.orientation

        self.orientation = quat_to_euler(msg.orientation)
    
    def depth_control_callback(self, msg):
        self.depth.set_target(msg.data)

    # Publishers

    def publish_vel(self):
        # Make the message
        msg = Twist()
        msg.linear.x = self.linear_vel[0]
        msg.linear.y = self.linear_vel[1]
        msg.linear.z = self.linear_vel[2]

        msg.angular.x = self.angular_vel[0]
        msg.angular.y = self.angular_vel[1]
        msg.angular.z = self.angular_vel[2]

        self.mover_pub.publish(msg)

    # Helper functions

    def get_parameters(self):
        self.declare_parameter('autodepth.kp', 0.4)
        self.declare_parameter('autodepth.ki', 0.8)
        self.declare_parameter('autodepth.kd', 0.1)
        self.declare_parameter('autodepth.min_depth', -0.3)
        self.declare_parameter('autodepth.max_depth', -1.0)
        self.declare_parameter('autodepth.tolerance', 0.02)

        # Depth parameters

        params = dict()
        params["autodepth.kp"] = self.get_parameter('autodepth.kp').value
        params["autodepth.ki"] = self.get_parameter('autodepth.ki').value
        params["autodepth.kd"] = self.get_parameter('autodepth.kd').value
        params["autodepth.min_depth"] = self.get_parameter('autodepth.min_depth').value
        params["autodepth.max_depth"] = self.get_parameter('autodepth.max_depth').value
        params["autodepth.tolerance"] = self.get_parameter('autodepth.tolerance').value

        return params
    
class PlannerState(Enum):
    Searching = 1  # Attempting to find a goal to follow
    Following = 2  # Following a goal that is in sight
    Avoiding = 3   # Avoiding a collision

class ControlState(Enum):
    Paused = 0
    Manual = 1          # Manual control
    ManualToAuto = 2    # Transitioning from manual to autonomous control
    Auto = 3            # Using autonomous functionality
    AutoToManual = 4    # Transitioning from autonomous to manual control

def main(args=None):
    rclpy.init(args=args)

    planner = Planner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        rclpy.try_shutdown()
        planner.destroy_node()