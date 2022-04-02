import rclpy
from std_msgs.msg import Int32


class ScoreboardLcdDriver:

    def init(self, webots_node, properties):
        try:
            rclpy.init(args=None)
        except Exception:  # noqa: E501
            # logging.exception("ScoreboardLcdDriver")
            pass  # noqa: E501

        self.__node = rclpy.node.Node('webots_match_start_driver')
        self.__subscriber = self.__node.create_subscription(
            Int32,
            '/scoreboard',
            self.listener_callback,
            1
        )

        self.__score = 0

    def listener_callback(self, msg):
        self.__node.get_logger().info('SCOREBOARD: I heard: "%s"' % msg.data)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
