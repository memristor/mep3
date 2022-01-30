import time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from mep3_msgs.action import DynamixelCommand
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import rclpy
import rclpy.node


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

    def __goal_callback(self, _):
        return GoalResponse.ACCEPT

    def __cancel_callback(self, _):
        return CancelResponse.ACCEPT

    async def __execute_callback(self, goal_handle):
        self.__motor.setPosition(goal_handle.request.position)
        self.__motor.setVelocity(0.01)
        result = DynamixelCommand.Result()

        while abs(self.__encoder.getValue() - goal_handle.request.position) > 0.1:
            if goal_handle.is_cancel_requested:
                # Return failure
                result.result = 1
                goal_handle.canceled()
                return DynamixelCommand.Result()

            time.sleep(0.1)

        # Return sucesss
        goal_handle.succeed()
        result.result = 0

        return result

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0, executor=self.__executor)
