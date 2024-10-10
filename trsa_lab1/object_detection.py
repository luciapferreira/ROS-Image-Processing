import rclpy
import cv2
import os, ament_index_python
from message_filters import Subscriber, TimeSynchronizer
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt 

from cv_bridge import CvBridge

class ObjectDetection(Node):

    def __init__(self):
        super().__init__('object_detection')

        self.bridge = CvBridge()
        self.sub_image_rect = Subscriber(self, Image, '/camera/image_rect')
        self.sub_image_processed = Subscriber(self, Image, '/camera/image_processed')
        self.pub_object_detection = self.create_publisher(Image, '/camera/object_detection', 10)

        # Sincronize both publishers        
        self.sync = TimeSynchronizer([self.sub_image_rect, self.sub_image_processed], 10)
        self.sync.registerCallback(self.object_detection_callback)

    def object_detection_callback(self, image_rect, image_processed):
        cv_image_rect = self.bridge.imgmsg_to_cv2(image_rect)
        cv_image_processed = self.bridge.imgmsg_to_cv2(image_processed)

        # Detecting and Creating Contours
        dilated = cv2.dilate(cv_image_processed, None, iterations=0)
        contours, hierarchy = cv2.findContours(cv_image_processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter Contours
        min_contour_area = 500  # Define your minimum area threshold
        filter_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]

        # Set minimum size threshold for bounding boxes
        min_width, min_height = 50, 50

        # Draw Bounding Boxes
        cv_image_object=cv_image_rect.copy()
        for cnt in filter_contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if w > min_width and h > min_height:
                cv_image_object = cv2.rectangle(cv_image_object, (x, y), (x+w, y+h), (0, 0, 255), 3)
 

        # Display the resulting frame
        cv2.imshow('Object Detected', cv_image_object)  
        cv2.waitKey(3)   

        self.pub_object_detection.publish(self.bridge.cv2_to_imgmsg(cv_image_object))
        

def main(args=None):
    rclpy.init(args=args)

    object_detection = ObjectDetection()

    rclpy.spin(object_detection)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    object_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
