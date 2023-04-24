import cv2

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

class Camera(Node):
    def __init__(self): # Default to 30fps
        super().__init__('camera')

        self.topic = "/nemo/image"

        # Fetch camera settings
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.res = (
            self.get_parameter('width').value,
            self.get_parameter('height').value,
        )
        
        self.declare_parameter('frame_rate', 30)
        self.frame_rate = self.get_parameter('frame_rate').value

        # Get video recording directory location parameter
        self.declare_parameter('video_dir', '/home/ros/recordings')
        self.video_dir = self.get_parameter('video_dir').get_parameter_value().string_value

        # Setup publisher
        self.publisher = self.create_publisher(CompressedImage, self.topic, 1) # Queue size of 1
        self.pub_timer = self.create_timer((1.0 / self.frame_rate), self.publish_image)

        # Setup OpenCV camera
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FPS, self.frame_rate)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.res[0]) 
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.res[1])

    def publish_image(self):
        # Retrieve image from camera
        ret, frame = self.camera.read()

        if not ret: # Can't publish due to error
            return

        # Handle publishing of frame
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()

        self.publisher.publish(msg)

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