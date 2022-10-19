#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from gpiozero import Button
from std_msgs.msg import Int8


class CinchDriver(Node):

    def __init__(self):
        super().__init__('cinch_driver')
        self.__publisher = self.create_publisher(
            Int8, '/match_start_status', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.__start_cinch = Button(5, pull_up=True, bounce_time=0.02)

    def process_cinch_state(self):
        self.__publisher.publish(Int8(data=0))
        rclpy.spin_once(self, timeout_sec=0)

        self.__start_cinch.wait_for_press()
        self.__publisher.publish(Int8(data=1))
        rclpy.spin_once(self, timeout_sec=0)
        

        self.__start_cinch.wait_for_release()
        self.__publisher.publish(Int8(data=2))
        rclpy.spin_once(self, timeout_sec=0)


def main(args=None):
    rclpy.init(args=args)

    cinch_driver = CinchDriver()
    cinch_driver.process_cinch_state()

    cinch_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
