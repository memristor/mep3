import logger

from mep3_msgs.action import VaccuumPumpCommand
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import rclpy.node

"""
# Move arm to position:
ros2 action send_goal /big/dynamixel_command/arm_right_motor_base mep3_msgs/action/DynamixelCommand "position: -90"  # noqa: E501
ros2 action send_goal /big/dynamixel_command/arm_right_motor_mid mep3_msgs/action/DynamixelCommand "position: 90"  # noqa: E501
ros2 action send_goal /big/dynamixel_command/arm_right_motor_gripper mep3_msgs/action/DynamixelCommand "position: 90"  # noqa: E501

# Test pump connect:
ros2 action send_goal /big/vaccuum_pump_command mep3_msgs/action/VaccuumPumpCommand "connect: 1"  # noqa: E501

# Move arm back:
ros2 action send_goal /big/dynamixel_command/arm_right_motor_mid mep3_msgs/action/DynamixelCommand "position: 0"  # noqa: E501

# Test pump disconnect:
ros2 action send_goal /big/vaccuum_pump_command mep3_msgs/action/VaccuumPumpCommand "connect: 0"  # noqa: E501
"""

class WebotsVaccuumPumpDriver:

    def init(self, webots_node, properties):
        try:
            rclpy.init(args=None)
        except Exception:
            logging.exception("WebotsVaccuumPumpDriver")
        self.__executor = MultiThreadedExecutor()

        namespace = properties['namespace']
        side = properties['side']

        self.__node = rclpy.create_node(f'webots_vaccuum_pump_driver_arm_{side}')
        self.__robot = webots_node.robot
        self.__connector = self.__robot.getDevice(f'arm_{side}_connector')
        timestep = int(self.__robot.getBasicTimeStep())
        self.__connector.enablePresence(timestep)

        self.__motor_action = ActionServer(
            self.__node,
            VaccuumPumpCommand,
            f'{namespace}/vaccuum_pump_command/arm_{side}_connector',
            execute_callback=self.__execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.__goal_callback,
            cancel_callback=self.__cancel_callback)

    def __goal_callback(self, _):
        return GoalResponse.ACCEPT

    def __cancel_callback(self, _):
        return CancelResponse.ACCEPT

    async def __execute_callback(self, goal_handle):
        connect = goal_handle.request.connect
        result = VaccuumPumpCommand.Result()

        if goal_handle.is_cancel_requested:
            result.result = 4  # other
            goal_handle.cancelled()

        if (connect):
            if self.__connector.isLocked():
                result.result = 1  # connected
                goal_handle.abort()
            else:
                if (self.__connector.getPresence()):
                    self.__connector.lock()
                    result.result = 1  # connected
                    goal_handle.succeed()
                else:
                    result.result = 3  # couldn't connect
                    goal_handle.abort()
        else:
            if self.__connector.isLocked():
                self.__connector.unlock()
                result.result = 0  # disconnected
                goal_handle.succeed()
            else:
                result.result = 0  # disconnected
                goal_handle.abort()

        return result

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0, executor=self.__executor)
