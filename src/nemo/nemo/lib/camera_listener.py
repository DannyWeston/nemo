import cv2
import numpy

from cv_bridge import CvBridge

from .video_recorder import VideoRecorder

from sensor_msgs.msg import CompressedImage

class CameraListener():
    def __init__(self, node, topic="/nemo/image", rate = 30, logger = None):
        self.topic = topic
        self.logger = logger

        self.rate = rate

        self.img = None

        # Allow for queue that is 1 second's worth of data
        self.sub = node.create_subscription(CompressedImage, self.topic, self.callback, self.rate)

        self.br = CvBridge()

        self.recorder = None

        self.record = True

    def callback(self, msg):
        self.img = self.br.compressed_imgmsg_to_cv2(msg)

        # Check for video recording
        if self.recorder is not None: 
            self.recorder.add_frame(self.img)

    def new_recording(self, dir):
        # Only make new recording if one not already there
        if self.recorder is None:
            if self.img is None: return # No images to provide size yet

            height, width, _ = self.img.shape
            self.recorder = VideoRecorder(dir=dir, size=(width, height), rate=self.rate)
        
    def stop_recording(self):
        if self.recorder is None: return

        self.recorder.stop_video()

    def shutdown(self):


        # Stop the video recorder
        self.stop_recording()