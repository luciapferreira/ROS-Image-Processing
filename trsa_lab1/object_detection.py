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

        # Creating Contours
        contours, hierarchy = cv2.findContours(cv_image_processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Set minimum and maximum size threshold for bounding boxes
        min_width, min_height = 250, 160
        max_width, max_height = 700, 700

        # Draw Bounding Boxes
        cv_image_object=cv_image_rect.copy()
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if min_width < w < max_width and min_height < h < max_height:
                cv_image_object = cv2.rectangle(cv_image_object, (x, y), (x+w, y+h), (0, 0, 255), 3)
                text = f"{w}x{h}"
                cv2.putText(cv_image_object, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)   

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
