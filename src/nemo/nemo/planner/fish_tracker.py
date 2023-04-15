import cv2

from cv_bridge import CvBridge

from ..lib.pid import PID

class FishTracker():
    def __init__(self):
        self.img = None

        self.br = CvBridge()

        self.follow_res = (160, 90)

        self.fish_colours_hsv_lower = [
            (100, 100, 0),  # Blue
            (35, 80, 80),   # Green
        ]

        self.fish_colours_hsv_upper = [
            (120, 255, 255),    # Blue
            (70, 255, 255),     # Green
        ]

        self.goal = 0
        
        self.follow_pid = PID(kp = 0.001)

    def img_callback(self, msg):
        self.img = self.br.compressed_imgmsg_to_cv2(msg)

    def search(self):
        if self.img is None: return None

        height, width, _ = self.img.shape

        threshold = (width * height) * 0.05 # Only 3% threshold required

        return self.check_for_goals(threshold)

    def check_for_goals(self, threshold, size=None):
        # Check if the image is marked to be resized
        img = self.img

        height, width, _ = img.shape
        if size is not None and size != (width, height):
            img = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
            width, height = size
            
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # find the colors within the specified boundaries and apply the masks
        mask = None
        for i in range(len(self.fish_colours_hsv_lower)):
            temp = cv2.inRange(img, self.fish_colours_hsv_lower[i], self.fish_colours_hsv_upper[i])

            if mask is None: mask = temp
            else: mask += temp

        masked_img = cv2.bitwise_and(img, img, mask=mask) # Apply mask

        masked_img = cv2.cvtColor(masked_img, cv2.COLOR_HSV2BGR) # Convert back to BGR

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) # Find contours

        # Finished if no contours were found
        if (len(contours) == 0):
            self.goal = 0
            return False

        # Check if the max-sized contour meets the threshold
        max_contour = max(contours, key=cv2.contourArea)
        if threshold > cv2.contourArea(max_contour): 
            self.goal = 0
            return False
            
        # Create a bounding box and return center 
        x, y, w, h = cv2.boundingRect(max_contour)

        # TODO: Account for vertical error on y-axis
        self.goal = (x + (w / 2.0)) - (width  / 2.0)

        return True

    def follow(self):
        # Reduce size of image for use in following
        # We don't need as much resolution to follow as we have already identified our target
        threshold = (self.follow_res[0] * self.follow_res[1]) * 0.05 # Only 5% threshold required
        return self.check_for_goals(img=self.img, threshold=threshold, size=self.follow_res)