import rclpy
from mep3_simulation import WebotsUserDriver
from std_msgs.msg import Int8


class MatchState:
    UNARMED = 0
    ARMED = 1
    STARTED = 2


class WebotsCinchDriver:

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__publisher = WebotsUserDriver.get().node.create_publisher(
            Int8, '/match_start_status', 1)
        self.__state = None

    def publish(self, state):
        if self.__state != state:
            self.__state = state
            self.__publisher.publish(Int8(data=self.__state))

    def step(self):
        if self.__state != MatchState.STARTED:
            elapsed_time = self.__robot.getTime()
            if elapsed_time <= 1.0:
                self.publish(MatchState.UNARMED)
            elif elapsed_time <= 2.0:
                self.publish(MatchState.ARMED)
            else:
                self.publish(MatchState.STARTED)

        rclpy.spin_once(WebotsUserDriver.get().node, timeout_sec=0, executor=WebotsUserDriver.get().executor)
