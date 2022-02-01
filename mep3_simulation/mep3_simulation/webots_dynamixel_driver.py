import time
from math import radians
import rclpy
import rclpy.node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from mep3_msgs.action import DynamixelCommand


DEFAULT_POSITION = radians(0) # deg
DEFAULT_VELOCITY = radians(45) # deg/s
DEFAULT_TOLERANCE = radians(1) # deg
DEFAULT_TIMEOUT = 5 # s


# Test: ros2 action send_goal /big/dynamixel_command/arm_right_motor_base mep3_msgs/action/DynamixelCommand "position: 2.2"
class WebotsDynamixelDriver:
    def init(self, webots_node, properties):
        try:
            rclpy.init(args=None)
        except Exception as e:
            pass
        self.__executor = MultiThreadedExecutor()

        namespace = properties['namespace']
        motor_name = properties['motorName']

        self.__node = rclpy.create_node(f'webots_dynamixel_driver_{motor_name}')
        self.__robot = webots_node.robot

        timestep = int(self.__robot.getBasicTimeStep())

        self.__motor = self.__robot.getDevice(motor_name)
        self.__encoder = self.__motor.getPositionSensor()
        self.__encoder.enable(timestep)
        self.__motor_action = ActionServer(
            self.__node,
            DynamixelCommand,
            f'{namespace}/dynamixel_command/{motor_name}',
            execute_callback=self.__execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.__goal_callback,
            cancel_callback=self.__cancel_callback
        )

    def __timeout_overflow(self, timeout):
        print("curr time: ", self.__robot.getTime())
        print("start time:", self.__start_time)
        if self.__robot.getTime() - self.__start_time > timeout:
            return True
        else:
            return False

    def __goal_callback(self, _):
        return GoalResponse.ACCEPT

    def __cancel_callback(self, _):
        return CancelResponse.ACCEPT

    async def __execute_callback(self, goal_handle):
        position = radians(goal_handle.request.position)
        velocity = radians(goal_handle.request.velocity)
        tolerance = radians(goal_handle.request.tolerance)
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
                goal_handle.canceled()
                return DynamixelCommand.Result()

            if goal_handle.is_cancel_requested:
                # Return failure
                result.result = 2
                goal_handle.canceled()
                return DynamixelCommand.Result()

            time.sleep(0.1)

        # Return sucesss
        goal_handle.succeed()
        result.result = 0

        return result

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0, executor=self.__executor)
