from geometry_msgs.msg import Twist

class MovementPublisher():
    def __init__(self, node, topic="/nemo/props", maxLinVel = (1.0, 1.0, 1.0), maxAngVel = (1.0, 1.0, 1.0)):
        self.maxLinVel = maxLinVel
        self.maxAngVel = maxAngVel

        self.velX = 0
        self.velY = 0
        self.velZ = 0

        self.velPitch = 0
        self.velRoll = 0
        self.velYaw = 0

        # Create publisher
        self.pub = node.create_publisher(Twist, topic, 10)

    def publish(self):
        # TODO: Clamp values within min/max

        # Publish clamped values
        msg = Twist()
        msg.linear.x = self.velX
        msg.linear.y = self.velY
        msg.linear.z = self.velZ

        msg.angular.x = self.velPitch
        msg.angular.y = self.velRoll
        msg.angular.z = self.velYaw

        self.pub.publish(msg)