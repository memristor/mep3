from math import pi, inf
from ranger_msgs.msg import Range
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class WebotsBinaryRangerDriver:
    """
    Binary rangers return 0 or 1, depending on the distance and threshold.

    In Webots, the distance rangers return the true distance from objects,
    whereas in the driver we convert them in binary rangers.
    In Webots we have four distance rangers:
    - distance_ranger_front_left
    - distance_ranger_front_right
    - distance_ranger_rear_left
    - distance_ranger_rear_right
    In ros2, we create four topics:
    - binary_ranger_command/front_left
    - binary_ranger_command/front_right
    - binary_ranger_command/rear_left
    - binary_ranger_command/rear_right
    """

    def init(self, webots_node, properties):
        try:
            rclpy.init(args=None)
        except Exception:  # noqa: E501
            pass  # noqa: E501
        self.__executor = MultiThreadedExecutor()

        self.__robot = webots_node.robot
        namespace = properties['namespace']
        ranger_position = properties['binaryRangerPosition']
        self.__distance_ranger = self.__robot.getDevice(
            f'distance_ranger_{ranger_position}')
        self.__node = rclpy.create_node(
            f'webots_binary_ranger_{ranger_position}')
        timestep = int(self.__robot.getBasicTimeStep())
        self.__distance_ranger.enable(timestep)
        self.__DISTANCE_THRESHOLD_M = 0.3
        self.__FOW_RAD = pi / 6
        self.__ranger_action = ActionClient(
            self.__node, Range,
            f'{namespace}/binary_ranger_command/{ranger_position}')

    def send_goal(self, range):
        # This is a fixed distance ranger, we could also write
        # min_range===max_range===distance

        goal_msg = Range.Goal()
        goal_msg.ratiation_type = 1  # ULTRASOUND=1 | INFRARED=1
        goal_msg.field_of_view = self.__FOW_RAD  # radians
        goal_msg.min_range = self.__DISTANCE_THRESHOLD_M  # meters
        goal_msg.max_range = inf
        goal_msg.range = range  # meters

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

    def step(self):
        distance = self.__distance_ranger.getValue()
        distance = distance <= self.__DISTANCE_TRESHOLD_M
        if distance:
            self.send_goal(self.__DISTANCE_THRESHOLD_M)
        else:
            self.send_goal(inf)
        rclpy.spin_once(self.__node, timeout_sec=0, executor=self.__executor)
