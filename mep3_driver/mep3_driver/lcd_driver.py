#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rpi_lcd import LCD

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

        self.lcd = LCD()
        self.__completed_tasks = set()
        self.__score = 0

    def display_pts(self, p):
        self.lcd.clear()
        self.lcd.text(str(p), 1)

    def listener_callback(self, msg):

        #if msg.task not in self.__completed_tasks:
        if True: # add always
            self.__score += msg.points
            self.__completed_tasks.add(msg.task)
            self.get_logger().info(
                "Added %i points for performing task '%s'." %
                (msg.points, msg.task)
            )

        else:
            self.get_logger().warn(
                "Not counting points for already performed task '%s'." %
                msg.task
            )

        self.display_pts(self.__score)
        self.get_logger().info(
            'Current score: %i points' % self.__score)
def main(args=None):
    rclpy.init(args=args)

    lcd_driver = LCDDriver()

    rclpy.spin(lcd_driver)

    lcd_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
