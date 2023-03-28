from .math import euler_to_quat, quat_to_euler

from nav_msgs.msg import Odometry

class OdomListener():
    def __init__(self, node, topic="/nemo/odom", rate=10, logger=None):
        self.logger = logger

        self.topic = topic

        self.rate = rate
        
        # Allow for queue that is 1 second's worth of data
        self.sub = node.create_subscription(Odometry, self.topic, self.callback, self.rate)

        self.posX = 0
        self.posY = 0
        self.posZ = 0

        self.rotX = 0
        self.rotY = 0
        self.rotZ = 0

    def callback(self, msg):
        # Extract information from msg
        self.posX = msg.pose.pose.position.x
        self.posY = msg.pose.pose.position.y
        self.posZ = msg.pose.pose.position.z

        pitch, roll, yaw = quat_to_euler(msg.pose.pose.orientation)
        self.rotX = pitch
        self.rotY = roll
        self.rotZ = yaw