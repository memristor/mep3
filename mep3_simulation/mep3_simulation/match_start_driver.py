import rclpy
from std_msgs.msg import Int8


class MatchState:
    UNARMED = 0
    ARMED = 1
    STARTED = 2


class WaitMatchStartDriver:

    def init(self, webots_node, properties):
        try:
            rclpy.init(args=None)
        except Exception:  # noqa: E501
            # logging.exception("WaitMatchStartDriver")
            pass  # noqa: E501

        self.__robot = webots_node.robot
        self.__node = rclpy.node.Node('webots_match_start_driver')
        self.__publisher = self.__node.create_publisher(
            Int8, '/match_start_status', 1)
        self.__state = None

    def publish(self, state):
        if self.__state != state:
            self.__state = state
            self.__publisher.publish(Int8(data=self.__state))

    def step(self):

        elapsed_time = self.__robot.getTime()

        if elapsed_time <= 1.0:
            self.publish(MatchState.UNARMED)
        elif elapsed_time <= 2.0:
            self.publish(MatchState.ARMED)
        else:
            self.publish(MatchState.STARTED)

        rclpy.spin_once(self.__node, timeout_sec=0)
