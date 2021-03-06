from mep3_msgs.msg import Scoreboard
from mep3_simulation import WebotsUserDriver


class ScoreboardLcdDriver:

    def init(self, webots_node, properties):
        self.__node = WebotsUserDriver.get().node
        self.__subscriber = self.__node.create_subscription(
            Scoreboard,
            '/scoreboard',
            self.listener_callback,
            1,
            callback_group=WebotsUserDriver.get().callback_group
        )

        self.__completed_tasks = set()
        self.__score = 0

    def listener_callback(self, msg):

        if msg.task not in self.__completed_tasks:
            self.__score += msg.points
            self.__completed_tasks.add(msg.task)
            self.__node.get_logger().info(
                "Added %i points for performing task '%s'." %
                (msg.points, msg.task)
            )

        else:
            self.__node.get_logger().warn(
                "Not counting points for already performed task '%s'." %
                msg.task
            )

        self.__node.get_logger().info(
            'Current score: %i points' % self.__score
        )

    def step(self):
        pass
