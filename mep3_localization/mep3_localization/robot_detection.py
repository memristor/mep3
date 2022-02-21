#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class DetectedRobots(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image, '/cam/cam_central/RasPi0', self.listener_callback, 10)
        self.subscription

        self.br = CvBridge()

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    detected_robots = DetectedRobots()
    rclpy.spin(detected_robots)
    detected_robots.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
