import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('camera_sub')

        self.topic = '/nemo/image'

        self.subscription = self.create_subscription(Image, self.topic, self.listener_callback, 30)
        self.subscription  # prevent unused variable warning
        
        self.get_logger().info(f'Subscribed to {self.topic} topic')

        self.br = CvBridge()

    def listener_callback(self, msg):
        current_frame = self.br.imgmsg_to_cv2(msg, 'rgba8')
        
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

        #cv2.imwrite("/home/ros/nemo/src/nemo_server/nemo_server/test.png", current_frame)
        
def main(args=None):
    rclpy.init(args=args)

    cameraSub = MinimalSubscriber()

    rclpy.spin(cameraSub)

    cameraSub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
