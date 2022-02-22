from enum import Enum

from std_msgs.msg import Int8
import rclpy
from rclpy.node import Node

class MatchState(Enum):
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

        rclpy.init(args=None)
        self.__robot = webots_node.robot
        self.__node = rclpy.node.Node('webots_match_start_driver')
        self.__publisher = self.__node.create_publisher(Int8, 'match_start_status', MatchState.UNARMED)

        return True

    def step(self):

        elapsed_time = self.__robot.getTime()

        if elapsed_time <= 1.0:
            self.__publisher.publish(MatchState.UNARMED)
        elif elapsed_time <= 2.0:
            self.__publisher.publish(MatchState.ARMED)
        else:
            self.__publisher.publish(MatchState.STARTED)

        rclpy.spin_once(self.__node, timeout_sec=0, executor=self.__executor)
