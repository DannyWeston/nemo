from .math import euler_from_quaternion

from .history_buffer import HistoryBuffer

class OdomListener():
    def __init__(self, logger = None):
        self.logger = logger

        self.topic = '/nemo/odom'
        
        self.odom_data_x = HistoryBuffer(50) # History buffer with queue size of 50
        self.odom_data_y = HistoryBuffer(50) # History buffer with queue size of 50
        self.odom_data_z = HistoryBuffer(50) # History buffer with queue size of 50
        self.odom_data_pitch = HistoryBuffer(50) # History buffer with queue size of 50
        self.odom_data_roll = HistoryBuffer(50) # History buffer with queue size of 50
        self.odom_data_yaw = HistoryBuffer(50) # History buffer with queue size of 50

    def callback(self, msg):
        # Extract information from msg
        x, y, z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        temp_rot = euler_from_quaternion(msg.pose.pose.orientation)
        pitch, roll, yaw = temp_rot[0], temp_rot[1], temp_rot[2]

        #self.logger.info(f'({x:.2f}, {y:.2f}, {z:.2f}) ({pitch:.2f}, {roll:.2f}, {yaw:.2f})')