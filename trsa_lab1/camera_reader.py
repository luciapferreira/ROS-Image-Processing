import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

class CameraReader(Node):

    def __init__(self):
        super().__init__('camera_reader')
        self.bridge = CvBridge()
        # Subcribe the /camera/image_raw topic
        self.sub_raw_frame = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.camera_reader_raw_callback,
            10
        )
        self.sub_frame = self.create_subscription(
            Image,
            "/camera/image_rect",
            self.camera_reader_callback,
            10
        )
        self.sub_image_rect = self.create_subscription(
            Image,
            "/camera/image_processed",
            self.camera_reader_processed_callback,
            10
        )

    def camera_reader_raw_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Camera Raw Feed", cv_image)
        cv2.waitKey(3)
    
    def camera_reader_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Camera Rectifier Feed", cv_image)
        cv2.waitKey(3)

    def camera_reader_processed_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow("Camera Processed Feed", cv_image)
        cv2.waitKey(3)


def main(args=None):
    rclpy.init(args=args)

    camera_reader = CameraReader()

    rclpy.spin(camera_reader)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
