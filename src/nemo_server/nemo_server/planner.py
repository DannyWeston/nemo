import time
import sys

import rclpy
from rclpy.node import Node

from .lib.states import ControlState, PlanningState, VideoRecordingState

from .lib.camera_listener import CameraListener
from .lib.goal_finder import GoalFinder
from .lib.states import StateManager

class Planner(Node):
    def __init__(self):
        super().__init__('planner')

        self.declare_parameter('default_video_directory', '/home/ros')

        self.camera_listener = CameraListener(self, logger=self.get_logger())

        self.goal_finder = GoalFinder()
        
        self.state_manager = StateManager(self, logger=self.get_logger())

        self.hz = 30
        self.it = 0
        
        self.create_timer(1.0 / self.hz, self.update)

    def update(self):
        # Check for movement updates
        if self.handle_control_state(): 
            self.handle_planner_state() # Auto movement

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

        if cstate is ControlState.Auto: return True

        if cstate is ControlState.Shutdown:
            self.shutdown()
            return False

        if cstate is ControlState.Paused:
            # Send pause signal to all movement-related things

            return False

        if cstate is ControlState.Manual:
            # Don't do anything as manual control is activated

            return False
            
        raise Exception("Unencounterable planner control state reached")

    def handle_planner_state(self):
        self.get_logger().info(f'Planner iteration {self.it}')

        # Should be in autononmous mode 
        self.it += 1

        # Hazards get priority
        self.check_hazards()

        if self.state_manager.planning_state is PlanningState.Hazard:
            self.avoid_hazard()

        elif self.state_manager.planning_state is (PlanningState.Searching or PlanningState.Idle):
            self.find_goal()

        elif self.state_manager.planning_state is PlanningState.Following:
            self.follow_goal()
        
        elif self.state_manager.planning_state is PlanningState.Success:
            self.goal_reached()

    def check_hazards(self):
        # Do something

        hazard = False
        if hazard: self.planning_state = PlanningState.Hazard

    def avoid_hazard(self):
        pass

    def follow_goal(self):
        pass

    def find_goal(self):
        pass

    def goal_reached(self):
        # TODO: Increment fish found counter, add to map etc.
        self.planning_state = PlanningState.Searching

    def check_goals(self):
        self.goal_finder.search(self.camera_listener.img)

    def shutdown(self):
        # Clearup
        self.camera_listener.shutdown()

    # def pub_masked_img(self):
    #     if self.masked_img is None: return

    #     msg = CompressedImage()
    #     msg.format = 'jpg'
    #     msg.data = np.array(cv2.imencode('.jpg', self.masked_img)[1]).tostring() # Encode to jpg
    #     self.publisher.publish(msg)

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