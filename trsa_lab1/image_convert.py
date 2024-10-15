import rclpy
import cv2
import os, ament_index_python
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

from cv_bridge import CvBridge

class ImageConvert(Node):

    def __init__(self):
        super().__init__('image_convert')

        # Declare parameters
        self.declare_parameter('threshold1', 30)
        self.declare_parameter('threshold2', 50)
        self.declare_parameter('max_kernel_length', 2)
        self.declare_parameter('max_kernel_gaussian', 21)
        
        self.threshold1 = self.get_parameter('threshold1').get_parameter_value().integer_value
        self.threshold2 = self.get_parameter('threshold2').get_parameter_value().integer_value
        self.max_kernel_length = self.get_parameter('max_kernel_length').get_parameter_value().integer_value
        self.max_kernel_gaussian = self.get_parameter('max_kernel_gaussian').get_parameter_value().integer_value
        

        self.bridge = CvBridge()
        self.sub_image_convert = self.create_subscription(Image, '/camera/image_rect', self.image_convert_callback , 10)
        self.pub_image_processed = self.create_publisher(Image, '/camera/image_processed', 10)

    def image_convert_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image_grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        kernel = np.ones((self.max_kernel_length, self.max_kernel_length), np.uint8)

        for i in range(1, self.max_kernel_gaussian, 2):
            cv_image_blur= cv2.GaussianBlur(cv_image_grey, (i, i), 0)

        cv_image_canny = cv2.Canny(cv_image_blur,self.threshold1,self.threshold2)
        
        # Dilate and Erode the picture
        cv_image_dilation = cv2.dilate(cv_image_canny, kernel, iterations=2) 
        cv_image_erosion = cv2.erode(cv_image_dilation, kernel, iterations=2) 
        
        self.pub_image_processed.publish(self.bridge.cv2_to_imgmsg(cv_image_erosion))
        

def main(args=None):
    rclpy.init(args=args)

    image_convert = ImageConvert()

    rclpy.spin(image_convert)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_convert.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
