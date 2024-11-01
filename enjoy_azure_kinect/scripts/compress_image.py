import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import argparse

TOPIC_NAME = '/rgb/image_raw'

class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor')

        self.declare_parameter('input_topic', TOPIC_NAME)
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        print(input_topic)

        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            CompressedImage,
            input_topic+'/compressed',
            10)
        self.bridge = CvBridge()

    def listener_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        ret, cv_compressed = cv2.imencode('.jpg', cv_image)
        if not ret:
            self.get_logger().warn('Image compression failed.')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(cv_compressed).tobytes()
        self.publisher.publish(msg)
        # self.get_logger().info('Published compressed image.')

def main(args=None):
    rclpy.init(args=args)
    image_compressor = ImageCompressor()
    rclpy.spin(image_compressor)
    image_compressor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
