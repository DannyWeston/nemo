from enum import Enum

from std_msgs.msg import UInt8

class StateManager():
    def __init__(self, node, control_topic = "/nemo/control", planning_topic = "/nemo/planning", vid_rec_topic = "/nemo/video/recording", rate = 1, logger=None):
        self.control_topic = control_topic
        self.planning_topic = planning_topic

        # Default state to paused
        self.control_state = ControlState.Auto
        self.planning_state = PlanningState.Searching
        self.vid_rec_state = VideoRecordingState.DoNotRecord

        # Only really need a low rate for control/planning subscribers
        self.control_sub = node.create_subscription(UInt8, control_topic, self.control_callback, rate)
        self.planning_sub = node.create_subscription(UInt8, planning_topic, self.planning_callback, rate)
        self.vid_rec_sub = node.create_subscription(UInt8, vid_rec_topic, self.vid_rec_callback, rate)
    
        self.logger = logger

    def control_callback(self, msg):
        state = int(msg.data) # From uint8 to int

        # TODO: if changing to manual control, the autonomous planner should be reset
        #       in order for safety

        self.control_state = ControlState(state) # Set internal state

    def planning_callback(self, msg):
        state = int(msg.data) # From uint8 to int

        self.planning_state = PlanningState(state) # Set internal state

    def vid_rec_callback(self, msg):
        state = int(msg.data) # From uint8 to int

        self.vid_rec_state = VideoRecordingState(state)

class ControlState(Enum):
    Paused = 0          # Stationary robot
    Shutdown = 1        # Used to signal a shutdown of the system
    Manual = 2          # Manually Controlled by user
    ManualToAuto = 3    # Transitioning from automatic to manual
    Auto = 4            # Using planner to find goals
    AutoToManual = 5    # Transitioning from manual to automatic

class PlanningState(Enum):
    Idle = 0       # Doing nothing
    Searching = 1  # Attempting to find a goal to follow
    Following = 2  # Following a goal that is in sight
    Success = 3    # Transitioning from a target that was met 
    Hazard = 4     # Avoiding a collision
    Avoiding = 5   # Avoiding a collision
    Aligning = 6

class VideoRecordingState(Enum):
    DoNotRecord = 0,
    Record = 1
