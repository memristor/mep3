import time

from mep3_msgs.action import VacuumPumpCommand
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from mep3_simulation import WebotsUserDriver

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

DISTANCE_TRESHOLD = 3.5e-3  # meters


class WebotsVacuumPumpDriver:

    def init(self, webots_node, properties):
        namespace = properties['namespace']
        connector_name = properties['connectorName']

        self.__node = rclpy.create_node(
            f'webots_vacuum_pump_driver_{connector_name}')
        self.__robot = webots_node.robot
        self.__connector = self.__robot.getDevice(f'{connector_name}')
        self.__distance_sensor = self.__robot.getDevice(
            f'{connector_name}_distance_sensor'
        )
        timestep = int(self.__robot.getBasicTimeStep())
        self.__connector.enablePresence(timestep)
        self.__distance_sensor.enable(timestep)

        self.__motor_action = ActionServer(
            WebotsUserDriver.get().node,
            VacuumPumpCommand,
            f'{namespace}/vacuum_pump_command/{connector_name}',
            execute_callback=self.__execute_callback,
            callback_group=WebotsUserDriver.get().callback_group,
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

        for _ in range(6):
            # Give it some time to connect
            distance = self.__distance_sensor.getValue()
            distance = distance <= DISTANCE_TRESHOLD
            if distance:
                break
            time.sleep(0.05)

        if connect:
            if self.__connector.isLocked():
                result.result = 3  # connected
                goal_handle.succeed()
            else:
                if self.__connector.getPresence() and distance:
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
        pass
