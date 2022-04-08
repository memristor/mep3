import struct
import can

from distutils import command
from functools import partial
from traceback import print_tb

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from threading import Lock, current_thread

from mep3_msgs.action import VacuumPumpCommand

VACUUM_PUMP_CAN_ID = 0x00006C10

VACUUMPUMPS = [
    {'id': 4, 'name': 'pump_left_hand'},
    {'id': 5, 'name': 'pump_right_hand'},
    ]
import rclpy
from rclpy.node import Node

class VacuumPump:
    def __init__(self, pump_id, pump_name):
        
        self.id = pump_id
        self.name = pump_name





class VacuumPumpDriver(Node):
    def __init__(self, pump, can_mutex, can_bus):
        super().__init__('Vacuum_pump' + str(pump['id']))
        self.__actions = []
        self.pump_list = []
        self.bus = can_bus

        new_pump = VacuumPump(
            pump_id=pump['id'], pump_name=pump['name']
        )
        self.pump_list.append(new_pump)

        self.__actions.append(
            ActionServer(
                self,
                VacuumPumpCommand,
                f'vacuum_command/{pump["name"]}',
                execute_callback=partial(self.__handle_vacuum_pump_command, vacuum_pump=new_pump),
                callback_group=ReentrantCallbackGroup(),
                goal_callback=self.__goal_callback,
                cancel_callback=self.__cancel_callback
            )
        )

    def __goal_callback(self, _):
        return GoalResponse.ACCEPT

    def __cancel_callback(self, _):
        return CancelResponse.ACCEPT

    def __handle_vacuum_pump_command(self, goal_handle,
                                   vacuum_pump: VacuumPump = None):
        self.get_logger().info(str(goal_handle.request))
        self.get_logger().info("Vacuum pump ID: " + str(vacuum_pump.id))
        self.get_logger().info("Thread: " + str(current_thread().name))

        connect = goal_handle.request.connect

        result = VacuumPumpCommand.Result()

        pump_id = VACUUM_PUMP_CAN_ID + vacuum_pump.id
        

        bin_data = struct.pack('b', connect)

        msg = can.Message(arbitration_id=pump_id, 
                         data=bin_data,
                         is_extended_id=True)

        try:
            self.bus.send(msg)
        except can.CanError:
            self.get_logger().info("CAN ERROR: Cannot send message over CAN bus. Check if can0 is active.")


        result.result = connect  # TODO: check sensor board?
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)

    can_mutex = Lock()
    bus = can.ThreadSafeBus(bustype='socketcan', channel='can0', bitrate=500000)

    # Set filters for receiving data
    bus.set_filters(filters=[{"can_id": VACUUM_PUMP_CAN_ID, "can_mask": 0x1FFFFFFF, "extended": True}])
    executor = MultiThreadedExecutor()
    for vacuum_pump in VACUUMPUMPS:
        driver = VacuumPumpDriver(vacuum_pump, can_mutex, bus)
        executor.add_node(driver)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
