import cv2
import numpy

class GoalFinder():
    def __init__(self):
        self.follow_res = (160, 90)

        self.fish_colours_hsv_lower = [
            (100, 100, 0),  # Blue
            (35, 80, 80),   # Green
        ]

        self.fish_colours_hsv_upper = [
            (120, 255, 255),    # Blue
            (70, 255, 255),     # Green
        ]

        self.masked_img = None

    def search(self, img):
        # Reduce size of image for use in movement prediction
        img = cv2.resize(self.camera_listener.img, self.follow_res, interpolation=cv2.INTER_NEAREST)

        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # find the colors within the specified boundaries and apply the masks
        
        mask = None
        for i in range(len(self.fish_colours_hsv_lower)):
            temp = cv2.inRange(img, self.fish_colours_hsv_lower[i], self.fish_colours_hsv_upper[i])

            if mask is None: mask = temp
            else: mask += temp

        masked_img = cv2.bitwise_and(img, img, mask=mask) # Apply mask

        self.masked_img = cv2.cvtColor(masked_img, cv2.COLOR_HSV2BGR)
