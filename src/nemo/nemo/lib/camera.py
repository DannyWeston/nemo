import cv2
from cv_bridge import CvBridge

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

class Camera(Node):
    def __init__(self): # Default to 30fps
        super().__init__('camera_publisher')

        self.br = CvBridge()

        self.raw_image = None

        # Fetch resolution setting
        self.declare_parameter('pixel_width', 640)
        self.declare_parameter('pixel_height', 480)

        self.res = (
            self.get_parameter('pixel_width').get_parameter_value().value,
            self.get_parameter('pixel_height').get_parameter_value().value,
        )

        # Fetch frame rate setting
        self.declare_parameter('frame_rate', 30)

        self.rate = self.get_parameter('frame_rate').get_parameter_value().value

        # Fetch topic setting
        self.declare_parameter('topic', '/nemo/image')

        self.topic = self.get_parameter('pixel_height').get_parameter_value().string_value

        # Setup publisher
        self.publisher = self.create_publisher(CompressedImage, self.topic, 1)
        self.pub_timer = self.create_timer((1.0 / self.rate), self.publish_image)

        # Setup OpenCV camera
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FPS, self.rate)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.res[0]) 
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.res[1])

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