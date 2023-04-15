import os
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt8

from enum import Enum

from datetime import datetime

class Recorder(Node):
    def __init__(self):
        super().__init__('recorder')
        
        self.img_topic = "/nemo/image"
        self.record_topic = "/nemo/recorder"

        self.br = CvBridge()

        self.state = VideoRecordingState.DoNotRecord # Default to not record

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('video_dir', '/home/ros/recordings')

        self.res = (self.get_parameter('width').value, self.get_parameter('height').value)
        self.dir = self.get_parameter('video_dir').get_parameter_value().string_value
        self.frame_rate = self.get_parameter('frame_rate').value

        self.fourcc = cv2.VideoWriter_fourcc(*"avc1")

        self.video = None
        self.file = None

        # Image subscriber
        self.img_sub = self.create_subscription(CompressedImage, self.img_topic, self.img_callback, 1) # Queue size of 1

        # Record subscriber
        self.record_sub = self.create_subscription(UInt8, self.record_topic, self.record_callback, 1) # Queue size of 1

    def img_callback(self, msg):
        if self.state is VideoRecordingState.Record and self.video:
            img = cv2.resize(self.br.compressed_imgmsg_to_cv2(msg), self.res)
            self.video.write(img)

    def record_callback(self, msg):
        new_state = VideoRecordingState(msg.data) # From uint8 to int

        match new_state:
            case VideoRecordingState.Record:
                if self.state is VideoRecordingState.DoNotRecord:
                    # No video already in progress, make new video
                    self.file = os.path.join(self.dir, f'{datetime.now().strftime("%d%m%Y_%H%M%S")}.mp4')
                    self.video = cv2.VideoWriter(self.file, self.fourcc, self.frame_rate, self.res)
                    self.state = new_state

            case VideoRecordingState.DoNotRecord:
                if self.state is VideoRecordingState.Record:
                    # Video in progress, stop video
                    self.video.release()
                    self.video = None
                    self.state = new_state

            case _:
                self.get_logger().info("Unknown video recording state received")

class VideoRecordingState(Enum):
    DoNotRecord = 0
    Record = 1

def main(args=None):
    rclpy.init(args=args)

    recorder = Recorder()

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        rclpy.try_shutdown()
        recorder.destroy_node()