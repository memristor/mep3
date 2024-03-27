#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import can

from mep3_msgs.msg import Scoreboard


class LCDDriver(Node):

    def __init__(self):

        super().__init__('lcd_driver_subscriber')
        self.subscription = self.create_subscription(
            Scoreboard,
            '/scoreboard',
            self.listener_callback,
            10
        )
        self.__score = 0
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')

    def send_can_message(self, arbitration_id, data):
        message = can.Message(arbitration_id=arbitration_id, data=data)
        try:
            self.bus.send(message)
            self.get_logger().info("Points sent successfully.")
        except can.CanError:
            self.get_logger().error("Failed to send points message.")

    def listener_callback(self, msg):
        score = msg.points
        self.__score += score
        self.send_can_message(0x6d20, [self.__score])


def main(args=None):
    rclpy.init(args=args)

    lcd_driver = LCDDriver()

    rclpy.spin(lcd_driver)

    lcd_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
