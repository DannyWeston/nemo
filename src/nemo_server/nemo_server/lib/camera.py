import cv2
from cv_bridge import CvBridge

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

class Camera(Node):
    def __init__(self, topic='/nemo/image', res=(640, 480), rate=30): # Default to 30fps
        # TODO: Setup loading parameters from config file

        super().__init__('camera_publisher')

        self.br = CvBridge()

        self.topic = topic

        self.rate = rate
        
        self.publisher = self.create_publisher(CompressedImage, topic, rate)

        self.pub_timer = self.create_timer((1.0 / rate), self.publish_image)

        self.raw_image = None

        self.camera = cv2.VideoCapture(0)

        # Default to 640x480 at 30fps
        self.camera.set(cv2.CAP_PROP_FPS, rate)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, res[0]) 
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])

    def publish_image(self):
        self.raw_image = self.get_image() 

        if self.raw_image is None: 
            return # Can't publish an image that doesn't exist
        
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', self.raw_image)[1]).tostring()

        self.publisher.publish(msg)

    def get_image(self):
        ret, frame = self.camera.read()

        if not ret: return None # Failed to capture image from feed
        
        return frame

def main(args=None):
    rclpy.init(args=args)

    camera = Camera()

    try:
        rclpy.spin(camera)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        camera.shutdown()
        rclpy.try_shutdown()
        camera.destroy_node()