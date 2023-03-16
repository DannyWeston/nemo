import rclpy
from rclpy.node import Node

from sensor_msgs.msg import FluidPressure
from cv_bridge import CvBridge
import cv2

class PressureSub(Node):

    def __init__(self):
        super().__init__('pressure_sub')

        self.topic = '/pressure'

        self.subscription = self.create_subscription(FluidPressure, self.topic, self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        
        self.get_logger().info(f'Subscribed to {self.topic} topic\n')

        self.br = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Pressure update incoming\n')

        self.logger.info(f'Pressure reading: {msg.fluid_pressure}')
        
def main(args=None):
    rclpy.init(args=args)

    pressureSub = PressureSub()

    rclpy.spin(pressureSub)

    pressureSub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
