import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageReader(Node):
    def __init__(self):
        super().__init__('image_reader')

        self.bridge = CvBridge()

        self.subog = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.original_callback,
            10)
        self.subog

        self.subproc = self.create_subscription(
            Image,
            '/camera/image_processed',
            self.processed_callback,
            10)
        self.subproc

    def original_callback(self, msg):
        #self.get_logger().info('Image frame received')

        cv_image = self.bridge.imgmsg_to_cv2(msg)

        cv2.imshow("Original camera", cv_image)
        cv2.waitKey(1)

    def processed_callback(self, msg):
        #self.get_logger().info('Image frame received')

        cv_image = self.bridge.imgmsg_to_cv2(msg)

        cv2.imshow("Processed image", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    image_reader = ImageReader()

    rclpy.spin(image_reader)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()