import rclpy
from rclpy.node import Node

import time

import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage

from .lib.camera_listener import CameraListener

class Planner(Node):

    def __init__(self):
        super().__init__('planner')

        self.camera_listener = CameraListener(self.get_logger(), self.new_image)
        self.camera_sub = self.create_subscription(CompressedImage, self.camera_listener.topic, self.camera_listener.callback, 30)

        self.publisher = self.create_publisher(CompressedImage, '/nemo/masked_image', 10)
        self.pub_timer = self.create_timer(1, self.pub_masked_img)

        self.fish_colours_hsv_lower = [
            (100, 150, 0),  # Blue
            (35, 80, 80),   # Green
        ]

        self.fish_colours_hsv_upper = [
            (120, 255, 255),    # Blue
            (70, 255, 255),     # Green
        ]

        # Implement listener for position

    def pub_masked_img(self):
        msg = CompressedImage()
        
    def new_image(self, img):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # find the colors within the specified boundaries and apply
        # the mask

        final_mask = None
        for i in range(len(self.fish_colours_hsv_lower)):
            mask = cv2.inRange(hsv_img, self.fish_colours_hsv_lower[i], self.fish_colours_hsv_upper[i])

            if (final_mask is None): final_mask = mask
            else: final_mask += mask

        masked_img = cv2.bitwise_and(hsv_img, hsv_img, final_mask)

        bgr_masked_img = cv2.cvtColor(masked_img, cv2.COLOR_HSV2BGR)

        # Check masked image
        cv2.imwrite("/home/ros/nemo/test.png", bgr_masked_img)

        #     ret,thresh = cv2.threshold(mask, 40, 255, 0)
        # if (cv2.__version__[0] > 3):
        #     contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # else:
        #     im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # if len(contours) != 0:
        #     # draw in blue the contours that were founded
        #     cv2.drawContours(output, contours, -1, 255, 3)

        #     # find the biggest countour (c) by the area
        #     c = max(contours, key = cv2.contourArea)
        #     x,y,w,h = cv2.boundingRect(c)

        #     # draw the biggest contour (c) in green
        #     cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),2)

        # # show the images
        # cv2.imshow("Result", np.hstack([image, output])) """

def main(args=None):
    time.sleep(3) # Sleep for 3 seconds on boot

    rclpy.init(args=args)

    planner = Planner()

    rclpy.spin(planner)

    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()