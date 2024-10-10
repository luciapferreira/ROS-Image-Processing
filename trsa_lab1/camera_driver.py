import rclpy
import cv2
import os, ament_index_python
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        # Intialize camera0 (/dev/video0 by default) 
        #self.cap = cv2.VideoCapture(0)

        # Use Mov Instead
        calibration_path = os.path.join(
            ament_index_python.get_package_share_directory('trsa_lab1'), 'video','test.mov')
        self.cap = cv2.VideoCapture(calibration_path)

        self.bridge = CvBridge()
        # Create the publisher camera0 to /camera/image_raw
        self.pub_frame = self.create_publisher(Image, '/camera/image_raw', 10)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        # Read the frame of camera 0
        ret, frame = self.cap.read()
        # Verify the frame was read sucefull and pusblish it in bgr8 format to publisher
        if ret == True:
            self.pub_frame.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    rclpy.spin(camera_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
