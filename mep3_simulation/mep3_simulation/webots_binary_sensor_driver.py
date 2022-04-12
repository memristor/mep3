from mep3_msgs.action import DistanceSensorReading
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class WebotsBinarySensorDriver:

    def init(self, webots_node, properties):
        try:
            rclpy.init(args=None)
        except Exception:  # noqa: E501
            pass  # noqa: E501
        self.__executor = MultiThreadedExecutor()

        self.__robot = webots_node.robot
        self.__sensor_front_left = self.__robot.getDevice(
            'distance_sensor_front_left')
        self.__sensor_front_right = self.__robot.getDevice(
            'distance_sensor_front_right')
        self.__sensor_back_left = self.__robot.getDevice(
            'distance_sensor_back_left')
        self.__sensor_back_right = self.__robot.getDevice(
            'distance_sensor_back_right')
        self.__node = rclpy.create_node('webots_binary_sensor_driver_node')

    def step(self):
        """
        Check in every time step if replica should be decoupled.

        If the angle of the motor is above a threshold,
        decouple the replica.
        The threshold is currently set at 80 degrees.
        """

        rclpy.spin_once(self.__node, timeout_sec=0, executor=self.__executor)
