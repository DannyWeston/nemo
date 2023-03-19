import rclpy
from rclpy.node import Node

import time

import cv2

from sensor_msgs.msg import FluidPressure, CompressedImage
from nav_msgs.msg import Odometry

from .lib.camera_listener import CameraListener
from .lib.pressure_listener import PressureListener
from .lib.odom_listener import OdomListener

class Localisation(Node):

    def __init__(self):
        super().__init__('localiser')

        self.camera_listener = CameraListener(self.get_logger(), self.new_image)
        self.pressure_listener = PressureListener(self.get_logger())
        self.odom_listener = OdomListener(self.get_logger())

        self.camera_sub = self.create_subscription(CompressedImage, self.camera_listener.topic, self.camera_listener.callback, 30)
        self.pressure_sub = self.create_subscription(FluidPressure, self.pressure_listener.topic, self.pressure_listener.callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_listener.topic, self.odom_listener.callback, 10)

    def new_image(self, img):
        #cv2.imwrite("/home/ros/nemo/test.png", img)
        pass
        
def main(args=None):
    time.sleep(3) # Sleep for 3 seconds on boot

    rclpy.init(args=args)

    localiser = Localisation()

    rclpy.spin(localiser)

    localiser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()