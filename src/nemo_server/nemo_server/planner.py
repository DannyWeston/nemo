import time

import rclpy
from rclpy.node import Node

from .lib.states import ControlState, PlanningState, VideoRecordingState

from .lib.pid import PID

from .lib.camera_listener import CameraListener
from .lib.odom_listener import OdomListener

from .lib.movement import MovementPublisher

from .lib.localiser import Localiser

from .lib.goal_finder import GoalFinder
from .lib.states import StateManager

class Planner(Node):
    def __init__(self):
        super().__init__('planner')

        self.declare_parameter('default_video_directory', '/home/ros')

        self.camera_listener = CameraListener(self, logger=self.get_logger())

        self.localiser = Localiser(self, logger=self.get_logger())

        self.mover = MovementPublisher(self)

        self.goal_finder = GoalFinder(logger=self.get_logger())

        self.state_manager = StateManager(self, logger=self.get_logger())

        self.align_pid = PID(kp = 0.001)

        self.target_depth = 0.3 # Target depth of 0.3 metres
        self.depth_pid = PID(kp = 2, ki = 0.6, kd = 0.3, min=-1.0, max=1.0)

        self.hz = 30
        self.it = 0
        
        self.create_timer(1.0 / self.hz, self.update)

    def update(self):
        # Check for movement updates
        self.handle_control_state()

        # Check for video recording updates
        self.handle_recording_state()

    def handle_recording_state(self):
        # TODO: Move this to hardware implementation file for both simulator and real-life implementation
        vrstate = self.state_manager.vid_rec_state
        if vrstate is VideoRecordingState.Record:
            dir = self.get_parameter('default_video_directory').get_parameter_value().string_value
            
            self.camera_listener.new_recording(dir)
    
    def handle_control_state(self): # Return False if not in automatic mode
        cstate = self.state_manager.control_state

        if cstate is ControlState.ManualToAuto:
            # Transitioning to automatic control
            self.transition_to_auto()
            return
        
        if cstate is ControlState.AutoToManual:
            # Transitioning to automatic control
            self.transition_to_manual()
            return

        if cstate is ControlState.Auto: 
            self.handle_planner_state()
            return

        if cstate is ControlState.Shutdown:
            self.shutdown()
            return

        if cstate is ControlState.Paused:
            # Send pause signal to all movement-related things
            return

        if cstate is ControlState.Manual:
            # Don't do anything as manual control is activated
            return
            
        raise Exception("Impossible planner control state reached")

    def transition_to_manual(self):
        self.mover.zero_vel()

        self.mover.publish()

        self.state_manager.control_state = ControlState.Manual

    def transition_to_auto(self):
        self.mover.zero_vel()

        self.mover.publish()

        self.state_manager.control_state = ControlState.Auto

    def handle_planner_state(self):
        # Autononmous mode
        self.mover.zero_vel()

        # Hazards get priority
        self.check_hazards()

        # Maintain PID depth
        self.adjust_depth()

        if self.state_manager.planning_state is PlanningState.Hazard:
            self.avoid_hazard()

        elif self.state_manager.planning_state is (PlanningState.Searching or PlanningState.Idle):
            self.search_goal()
        
        elif self.state_manager.planning_state is PlanningState.Aligning:
            self.align_goal()

        elif self.state_manager.planning_state is PlanningState.Following:
            self.follow_goal()
        
        elif self.state_manager.planning_state is PlanningState.Success:
            self.goal_reached()

        self.mover.publish()

    def check_hazards(self):
        hazard = False
        if hazard: self.planning_state = PlanningState.Hazard

    def avoid_hazard(self):
        self.get_logger().info("Avoiding a hazard")

    def adjust_depth(self):
        _, depth, _ = self.localiser.get_position()

        error = self.target_depth - depth

        self.mover.velY = -self.depth_pid.update(error)

    def follow_goal(self):
        if self.camera_listener.img is None: return # Can't follow a goal with no image

        if not self.goal_finder.follow(self.camera_listener.img):
            # Lost track of goal so look for new one
            self.state_manager.planning_state = PlanningState.Searching
            return

        self.get_logger().info("Following identified goal...")

        # Drive towards goal at 0.2 m/s
        self.mover.velX = 0.2

    def search_goal(self):
        if self.camera_listener.img is None: return # Can't find a goal with no image
       
        if self.goal_finder.search(self.camera_listener.img):
            # Found a goal, rotate change state
            self.state_manager.planning_state = PlanningState.Aligning
            return

        # Didn't find a goal, continue random search

    def goal_reached(self):
        self.get_logger().info("Reached a goal")

        # TODO: Increment fish found counter, add to map etc.
        self.planning_state = PlanningState.Searching

    def align_goal(self):
        if self.camera_listener.img is None: return # Can't align to a goal whenw e have no image

        if not self.goal_finder.search(self.camera_listener.img):
            # Could no longer find the goal, change back to search
            self.state_manager.planning_state = PlanningState.Searching
            return
        
        if 4 < self.goal_finder.goal:
            # Facing goal so can now start following (reset align_pid to 0 for next usage)
            self.align_pid.reset()
            self.state_manager.planning_state = PlanningState.Following
            return
        
        # Pipe error of located goal into PID Controller
        pid_out = self.align_pid.update(error=self.goal_finder.goal)

        # Rotate to face goal
        self.mover.velZ = pid_out
        
    def shutdown(self):
        # Clearup
        self.camera_listener.shutdown()

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