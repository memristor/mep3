#!/usr/bin/env python3

import struct
import can
from functools import partial
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from threading import Lock, current_thread
from mep3_msgs.action import VacuumPumpCommand
import rclpy
from rclpy.node import Node


# ros2 action send_goal /big/vacuum_pump_command/arm_left_connector mep3_msgs/action/VacuumPumpCommand "connect: 0"


VACUUM_PUMP_CAN_ID = 0x00006C10
VACUUMPUMPS = [
    {'id': 5, 'name': 'arm_connector'}
]


class VacuumPumpDriver(Node):
    def __init__(self, can_mutex, can_bus):
        super().__init__('vacuum_pump_driver')
        self.__actions = []
        self.pump_list = []
        self.bus = can_bus

        callback_group = ReentrantCallbackGroup()
        for pump in VACUUMPUMPS:
            self.__actions.append(
                ActionServer(
                    self,
                    VacuumPumpCommand,
                    f'vacuum_pump_command/{pump["name"]}',
                    execute_callback=partial(self.__handle_vacuum_pump_command, pump_id=pump['id']),
                    callback_group=callback_group,
                    goal_callback=self.__goal_callback,
                    cancel_callback=self.__cancel_callback
                )
            )

    def __goal_callback(self, _):
        return GoalResponse.ACCEPT

    def __cancel_callback(self, _):
        return CancelResponse.ACCEPT

    def __handle_vacuum_pump_command(self, goal_handle, pump_id=None):
        self.get_logger().info(str(goal_handle.request))
        self.get_logger().info("Vacuum pump ID: " + str(pump_id))
        self.get_logger().info("Thread: " + str(current_thread().name))

        connect = goal_handle.request.connect

        result = VacuumPumpCommand.Result()

        pump_id = VACUUM_PUMP_CAN_ID + pump_id

        bin_data = struct.pack('b', connect)

        msg = can.Message(arbitration_id=pump_id,
                          data=bin_data,
                          is_extended_id=True)

        try:
            self.bus.send(msg)
        except can.CanError:
            self.get_logger().info("CAN ERROR: Cannot send message over CAN bus. Check if can0 is active.")

        result.result = 1
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)

    can_mutex = Lock()
    bus = can.ThreadSafeBus(bustype='socketcan', channel='can0', bitrate=500000)

    # Set filters for receiving data
    bus.set_filters(filters=[{'can_id': VACUUM_PUMP_CAN_ID, 'can_mask': 0x1FFFFFFF, 'extended': True}])
    executor = MultiThreadedExecutor()
    driver = VacuumPumpDriver(can_mutex, bus)
    executor.add_node(driver)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
