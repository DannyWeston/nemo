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

# Nemo Lib Imports
from .fish_tracker import FishTracker

from .depth import Depth

from ..lib.math import quat_to_euler


class Planner(Node):
    def __init__(self):
        super().__init__('planner')

        self.p_state = PlannerState.Searching # Default planner state is searching
        self.c_state = ControlState.Paused # Default control state is paused

        # Module for tracking fish
        self.tracker = FishTracker()
        self.camera_sub = self.create_subscription(CompressedImage, "/nemo/image", self.tracker.img_callback, 1) # Queue size of 1

        # Module for maintaining a set depth
        self.depth = Depth()
        self.target_sub = self.create_subscription(Float32, "/nemo/depth", self.depth.target_callback, 1) # Queue size of 1

        # Module for retrieving Nemo's location
        self.position = None
        self.orientation = None
        self.pose_sub = self.create_subscription(Pose, "/nemo/pose", self.pose_callback, 1) # Queue size of 1

        # Module for sending movement updates to the hardware
        self.linear_vel = [0.0, 0.0, 0.0]
        self.angular_vel = [0.0, 0.0, 0.0]
        self.mover_pub = self.create_publisher(Twist, "/nemo/props", 10)

        # Receive updates for planner state updates
        self.planner_sub = self.create_subscription(UInt8, "/nemo/planner", self.planner_callback, 1) # Queue size of 1

        # Receive updates for control state updates
        self.control_sub = self.create_subscription(UInt8, "/nemo/control", self.control_callback, 1) # Queue size of 1

        # Setup updates
        self.declare_parameter('rate', 10)
        self.rate = self.get_parameter('rate').value
        self.create_timer(1.0 / self.rate, self.update)

    def planner_callback(self, msg):
        self.p_state = PlannerState(int(msg.data))

    def control_callback(self, msg):
        self.c_state = ControlState(int(msg.data))

    def pose_callback(self, msg):
        self.position = (msg.position.x, msg.position.y, msg.position.z)

        self.orientation = quat_to_euler(msg.orientation)

    def publish(self):
        # Make the message
        msg = Twist()
        msg.linear.x = self.linear_vel[0]
        msg.linear.y = self.linear_vel[1]
        msg.linear.z = self.linear_vel[2]

        msg.angular.x = self.angular_vel[0]
        msg.angular.y = self.angular_vel[1]
        msg.angular.z = self.angular_vel[2]

        self.mover_pub.publish(msg)

    def zero_vel(self):
        self.linear_vel = [0.0, 0.0, 0.0]
        self.angular_vel = [0.0, 0.0, 0.0]

    def update(self):
        match self.c_state:
            case ControlState.ManualToAuto: # Transitioning to automatic control
                self.zero_vel()
                self.publish()
                self.get_logger().info("Changing to autonomous control")
                self.c_state = ControlState.Auto
            
            case ControlState.AutoToManual: # Transitioning to automatic control
                self.zero_vel()
                self.publish()
                self.get_logger().info("Changing to manual control")
                self.c_state = ControlState.Manual

            case ControlState.Auto:
                self.auto_update()

            case ControlState.Manual:
                # Do nothing
                pass

            case ControlState.Paused: # TODO: Ensure EVERYTHING is stopped
                self.zero_vel()
                self.publish()

            case _:
                self.zero_vel()
                self.publish()
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
                if self.tracker.search(): self.p_state = PlannerState.Following

            case PlannerState.Following:
                self.follow_goal()

        self.publish()

    def check_hazards(self):
        hazard = False
        if hazard: self.p_state = PlannerState.Avoiding

    def avoid_hazard(self):
        self.get_logger().info("Attempting to avoid a hazard...")

    def follow_goal(self):
        if not self.tracker.follow(): # Lost track of goal so look for new one
            self.p_state = PlannerState.Searching
            return

        self.get_logger().info("Following identified goal...")

        # Drive towards goal at 0.2 m/s
        self.linear_vel[1] = 0.2

    # def align_goal(self):
    #     pass
    #     if self.camera_listener.img is None: return # Can't align to a goal when we have no image

    #     if not self.goal_finder.search(self.camera_listener.img):
    #         # Could no longer find the goal, change back to search
    #         self.state_manager.planning_state = PlannerState.Searching
    #         return
        
    #     if 4 < self.goal_finder.goal:
    #         # Facing goal so can now start following (reset align_pid to 0 for next usage)
    #         self.align_pid.reset()
    #         self.state_manager.planning_state = PlannerState.Following
    #         return
        
    #     # Pipe error of located goal into PID Controller
    #     pid_out = self.align_pid.update(error=self.goal_finder.goal)

    #     # Rotate to face goal
    #     self.mover.velZ = pid_out
        
    def shutdown(self):
        pass

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
        planner.shutdown()
        rclpy.try_shutdown()
        planner.destroy_node()