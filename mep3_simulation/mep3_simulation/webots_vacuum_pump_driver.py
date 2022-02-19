import logging

from mep3_msgs.action import VacuumPumpCommand
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import rclpy.node
"""
# Move RIGHT arm to position:
ros2 action send_goal /big/dynamixel_command/arm_right_motor_base mep3_msgs/action/DynamixelCommand "position: -90"  # noqa: E501
ros2 action send_goal /big/dynamixel_command/arm_right_motor_mid mep3_msgs/action/DynamixelCommand "position: 90"  # noqa: E501
ros2 action send_goal /big/dynamixel_command/arm_right_motor_gripper mep3_msgs/action/DynamixelCommand "position: 90"  # noqa: E501

# Test RIGHT pump connect:
ros2 action send_goal /big/vacuum_pump_command/arm_right_connector mep3_msgs/action/VacuumPumpCommand "connect: 1"  # noqa: E501

# Move RIGHT arm back:
ros2 action send_goal /big/dynamixel_command/arm_right_motor_mid mep3_msgs/action/DynamixelCommand "position: 0"  # noqa: E501

# Test RIGHT pump disconnect:
ros2 action send_goal /big/vacuum_pump_command/arm_right_connector mep3_msgs/action/VacuumPumpCommand "connect: 0"  # noqa: E501
"""
"""
# Move LEFT arm to position:
ros2 action send_goal /big/dynamixel_command/arm_left_motor_base mep3_msgs/action/DynamixelCommand "position: 90"  # noqa: E501
ros2 action send_goal /big/dynamixel_command/arm_left_motor_mid mep3_msgs/action/DynamixelCommand "position: 90"  # noqa: E501
ros2 action send_goal /big/dynamixel_command/arm_left_motor_gripper mep3_msgs/action/DynamixelCommand "position: 90"  # noqa: E501

# Test LEFT pump connect:
ros2 action send_goal /big/vacuum_pump_command/arm_left_connector mep3_msgs/action/VacuumPumpCommand "connect: 1"  # noqa: E501

# Move LEFT arm back:
ros2 action send_goal /big/dynamixel_command/arm_left_motor_mid mep3_msgs/action/DynamixelCommand "position: 0"  # noqa: E501

# Test LEFT pump disconnect:
ros2 action send_goal /big/vacuum_pump_command/arm_left_connector mep3_msgs/action/VacuumPumpCommand "connect: 0"  # noqa: E501
"""


class WebotsVacuumPumpDriver:

    def init(self, webots_node, properties):
        try:
            rclpy.init(args=None)
        except Exception:
            # logging.exception("WebotsVacuumPumpDriver")
            pass  # noqa: E501
        self.__executor = MultiThreadedExecutor()

        namespace = properties['namespace']
        connector_name = properties['connectorName']

        self.__node = rclpy.create_node(
            f'webots_vacuum_pump_driver_{connector_name}')
        self.__robot = webots_node.robot
        self.__connector = self.__robot.getDevice(f'{connector_name}')
        timestep = int(self.__robot.getBasicTimeStep())
        self.__connector.enablePresence(timestep)

        self.__motor_action = ActionServer(
            self.__node,
            VacuumPumpCommand,
            f'{namespace}/vacuum_pump_command/{connector_name}',
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
        result = VacuumPumpCommand.Result()

        if goal_handle.is_cancel_requested:
            result.result = 4  # other
            goal_handle.cancelled()

        if connect:
            if self.__connector.isLocked():
                result.result = 3  # connected
                goal_handle.succeed()
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
                goal_handle.succeed()

        return result

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0, executor=self.__executor)
