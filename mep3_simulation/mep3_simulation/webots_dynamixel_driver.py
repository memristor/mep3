from math import radians
import time

from mep3_msgs.action import DynamixelCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from mep3_simulation import WebotsUserDriver

DEFAULT_POSITION = radians(0)  # deg
DEFAULT_VELOCITY = radians(45)  # deg/s
DEFAULT_TOLERANCE = radians(1)  # deg
DEFAULT_TIMEOUT = 5  # s
"""
# Test:
ros2 action send_goal /big/dynamixel_command/arm_right_motor_base mep3_msgs/action/DynamixelCommand "position: 2.2"  # noqa: E501
"""


class WebotsDynamixelDriver:

    def init(self, webots_node, properties):
        namespace = properties['namespace']
        motor_name = properties['motorName']

        if 'gearRatio' in properties:
            self.__gear_ratio = float(
                properties['gearRatio']
            )
        else:
            self.__gear_ratio = 1.0

        self.__robot = webots_node.robot
        timestep = int(self.__robot.getBasicTimeStep())

        self.__motor = self.__robot.getDevice(motor_name)
        self.__encoder = self.__motor.getPositionSensor()
        self.__encoder.enable(timestep)
        self.__motor_action = ActionServer(
            WebotsUserDriver.get().node,
            DynamixelCommand,
            f'{namespace}/dynamixel_command/{motor_name}',
            execute_callback=self.__execute_callback,
            callback_group=WebotsUserDriver.get().callback_group,
            goal_callback=self.__goal_callback,
            cancel_callback=self.__cancel_callback)

    def __timeout_overflow(self, timeout):
        if self.__robot.getTime() - self.__start_time > timeout:
            return True
        else:
            return False

    def __goal_callback(self, _):
        return GoalResponse.ACCEPT

    def __cancel_callback(self, _):
        return CancelResponse.ACCEPT

    async def __execute_callback(self, goal_handle):
        position = radians(goal_handle.request.position) * \
            self.__gear_ratio
        velocity = radians(goal_handle.request.velocity) * \
            self.__gear_ratio
        tolerance = radians(goal_handle.request.tolerance) * \
            self.__gear_ratio
        timeout = goal_handle.request.timeout

        self.__motor.setPosition(position)
        self.__motor.setVelocity(velocity if velocity else DEFAULT_VELOCITY)
        if not tolerance:
            tolerance = DEFAULT_TOLERANCE
        if not timeout:
            timeout = DEFAULT_TIMEOUT

        self.__start_time = self.__robot.getTime()
        result = DynamixelCommand.Result()

        while abs(self.__encoder.getValue() - position) > tolerance:
            if self.__timeout_overflow(timeout):
                # Return failure
                result.result = 1
                goal_handle.abort()
                return result

            if goal_handle.is_cancel_requested:
                # Return failure
                result.result = 2
                goal_handle.canceled()
                return result

            time.sleep(0.1)

        # Return sucesss
        goal_handle.succeed()
        result.result = 0

        return result

    def step(self):
        pass
