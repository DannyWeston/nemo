import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

import numpy as np

from sensor_msgs.msg import CompressedImage

from nemo_interfaces.msg import TrackerMsg

import os

SCORE_THRESHOLD = 0.2
NMS_THRESHOLD = 0.4
CONFIDENCE_THRESHOLD = 0.4

class Tracker(Node):
    def __init__(self):
        super().__init__("tracker")
        
        self.br = CvBridge()

        self.processed = True
        self.img = None
        self.found_coords = None
        self.found_id = None
        
        # Load Model
        self.declare_parameter('model_path', "data/model.onnx")
        self.model_path =  self.get_parameter('model_path').get_parameter_value().string_value
        self.model = cv2.dnn.readNet(self.model_path)

        # Load classes
        self.declare_parameter('classes_path', "data/classes.txt")
        self.classes_path = self.get_parameter('classes_path').get_parameter_value().string_value
        self.classes = self.load_classes(self.classes_path)


        self.declare_parameter('image_size.x', 640)
        self.declare_parameter('image_size.y', 640)
        self.input_size = (self.get_parameter('image_size.x').value, self.get_parameter('image_size.y').value)
        
        # Listen for camera updates
        self.camera_topic = "/nemo/image"
        self.camera_sub = self.create_subscription(CompressedImage, self.camera_topic, self.img_callback, 1) # Queue size of 1

        # Crate publisher
        self.tracker_pub = self.create_publisher(TrackerMsg, "/nemo/tracker", 1)

        # Set update rate
        self.declare_parameter('rate', 5) # Target of 5 fps
        self.rate = self.get_parameter('rate').value
        self.create_timer(1.0 / self.rate, self.update)

    def update(self):
        if self.processed: return

        # Process and publish image

        img = self.format_image(self.img)
        result = self.detect(img)

        ids, confidences, boxes = self.process_results(img, result[0])

        self.found_id, self.found_coords = self.find_target(ids, confidences, boxes)

        self.processed = True

        self.publish()

    def publish(self):
        if self.found_id and self.found_coords:
            msg = TrackerMsg()
            msg.id = self.found_id
            msg.x1 = self.found_coords[0]
            msg.y1 = self.found_coords[1]
            msg.x2 = self.found_coords[2]
            msg.y2 = self.found_coords[3]
            
            msg.width = self.img.shape[0]

            self.tracker_pub.publish(msg)

            self.found_id = None
            self.found_coords = None

    def img_callback(self, msg):
        self.img = self.br.compressed_imgmsg_to_cv2(msg)

        self.processed = False

    def detect(self, image):
        blob = cv2.dnn.blobFromImage(image, 1/255.0, self.input_size, swapRB=True, crop=False)
        self.model.setInput(blob)
        return self.model.forward()

    def load_classes(self, path):
        class_list = []
        with open(path, "r") as f:
            class_list = [cname.strip() for cname in f.readlines()]

        return class_list

    def process_results(self, input_image, output_data):
        class_ids = []
        confidences = []
        boxes = []

        rows = output_data.shape[0]

        image_width, image_height, _ = input_image.shape

        x_factor = image_width / self.input_size[0]
        y_factor =  image_height / self.input_size[1]

        for r in range(rows):
            row = output_data[r]
            confidence = row[4]
            if confidence >= 0.4:

                classes_scores = row[5:]
                _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
                class_id = max_indx[1]
                if (classes_scores[class_id] > .25):

                    confidences.append(confidence)

                    class_ids.append(class_id)

                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                    left = int((x - 0.5 * w) * x_factor)
                    top = int((y - 0.5 * h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    box = np.array([left, top, width, height])
                    boxes.append(box)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45) 

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes

    def format_image(self, img):
        x, y, _ = img.shape
        larger = max(x, y)

        result = np.zeros((larger, larger, 3), np.uint8)
        result[0:x, 0:y] = img

        return result

    def find_target(self, class_ids, confidences, boxes):
        min_area = None
        min_id = None
        coords = None

        for (id, _, box) in zip(class_ids, confidences, boxes):
            (x1, y1), (x2, y2) = (box[0], box[1]), (box[0] + box[2], box[1])
            area = (x2 - x1) * (y2 - y1)
            if not min_area or area < min_area:
                min_area = area
                min_id = self.classes[id]
                coords = [int(x1), int(y1), int(x2), int(y2)]
        
        return (min_id, coords)
    
def main(args=None):
    rclpy.init(args=args)

    tracker = Tracker()

    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        rclpy.try_shutdown()
        tracker.destroy_node()