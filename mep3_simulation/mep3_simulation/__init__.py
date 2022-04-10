import rclpy
import rclpy.node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class WebotsUserDriver:
    instance = None

    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.node.Node('webots_user_driver')
        self.callback_group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor()

    @staticmethod
    def get():
        if WebotsUserDriver.instance is None:
            WebotsUserDriver.instance = WebotsUserDriver()
        return WebotsUserDriver.instance
