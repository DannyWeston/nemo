import rclpy
from rclpy.node import Node

import os
import numpy
import cv2

from datetime import datetime

class VideoRecorder():
    def __init__(self, dir, size, rate = 30):
        self._size = size
        self._name = datetime.now().strftime("%d%m%Y_%H%M%S")

        self.dir = dir

        self.target_file = os.path.join(self.dir, self._name + ".mp4")

        self._fourcc = cv2.VideoWriter_fourcc(*"avc1")

        self.video = cv2.VideoWriter(self.target_file, self._fourcc, rate, size)

    def add_frame(self, frame):
        frame = cv2.resize(frame, self._size)
        
        self.video.write(frame)

    def stop_video(self):
        # Release the video
        self.video.release()
        self.video = None

        self._size = None
        self._name = None