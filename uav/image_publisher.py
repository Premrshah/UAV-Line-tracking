# image_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()

        # Load image from file
        image_path = "/home/ubuntu/ros2_ws/src/uav/image.jpg"
        self.cv_image = cv2.imread(image_path)

        if self.cv_image is None:
            self.get_logger().error('Image not found! Check the file path.')
        else:
            self.get_logger().info('Image loaded and ready to publish.')

    def timer_callback(self):
        if self.cv_image is not None:
            msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info('Published image to /camera/image_raw')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
