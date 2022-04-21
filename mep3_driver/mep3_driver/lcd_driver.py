import rclpy
from rclpy.node import Node

from mep3_msgs.action import Scoreboard


class LCDDriver(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Scoreboard,
            'scoreboard',
            self.listener_callback,
            10)

        self.total_points = 0

        from .lcd.lcddriver import lcd
        self.lcd = lcd()

    def display_pts(self, p):
        self.lcd.clear()
        self.lcd.display_string(str(p), 0)

    def listener_callback(self, msg):
        task_name = msg.data[0]
        points = int(msg.data[1])
        self.total_points += points
        self.get_logger().info(
            'Task: {} added {} pts for total of {}'.format(
                task_name, points, self.total_points
            ))


def main(args=None):
    rclpy.init(args=args)

    lcd_driver = LCDDriver()

    rclpy.spin(lcd_driver)

    lcd_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
