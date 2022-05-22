#!/usr/bin/env python3

import struct
import can
import math
from functools import partial
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from threading import Lock, current_thread
from mep3_msgs.action import ResistanceMeter
import rclpy
from rclpy.node import Node


RESISANCE_METER_CAN_ID = 0x00008D73
RESISANCE_METER_CAN_MASK = 0x1FFFFFFF
RESISTANCE_METERS = [
    {'name': 'left'},
    {'name': 'right'},
]


def readout_to_resistance(value):
    i = 0.7e-3  # ADC current
    c = 4096    # ADC resolution
    v = 3.3     # ADC maximum voltage
    return int(value / ((c / v) * i))


class ResistanceMeterDriver(Node):
    def __init__(self, can_mutex, can_bus):
        super().__init__('resistance_meter_driver')
        self.__actions = []
        self.meter_list = []

        self.can_mutex = can_mutex
        self.bus = can_bus

        for meter in RESISTANCE_METERS:
            self.__actions.append(
                ActionServer(
                    self,
                    ResistanceMeter,
                    f'resistance_meter/{meter["name"]}',
                    execute_callback=partial(self.__handle_resistance_meter),
                    goal_callback=self.__goal_callback,
                    cancel_callback=self.__cancel_callback
                )
            )

    def __goal_callback(self, _):
        return GoalResponse.ACCEPT

    def __cancel_callback(self, _):
        return CancelResponse.ACCEPT

    def __handle_resistance_meter(self, goal_handle):
        self.get_logger().info(str(goal_handle.request))
        self.get_logger().info("Resistance meter ID: " + str(RESISANCE_METER_CAN_ID))
        self.get_logger().info("Thread: " + str(current_thread().name))

        result = ResistanceMeter.Result()
        resistance = None

        msg = can.Message(
            arbitration_id=RESISANCE_METER_CAN_ID,
            data=[],  # Send an empty message
            is_extended_id=True
        )

        self.can_mutex.acquire()

        try:
            self.bus.send(msg)
        except can.CanError:
            self.get_logger().info(
                "CAN ERROR: Cannot send message over CAN bus. Check if can0 is active.")

        recv = self.bus.recv(timeout=1)
        print(recv)
        if recv is not None:
            readout = struct.unpack('<H', recv.data[:2])[0]
            self.get_logger().info("->>>>>>>>VALUE FROM CAN: " + str(readout))
            resistance = readout_to_resistance(readout)
            self.get_logger().info("Resistance: " + str(resistance))
            self.get_logger().info("can data resist: " + str(recv))

        self.can_mutex.release()

        if resistance is None:
            result.resistance = 0
            goal_handle.abort()
        else:
            goal_handle.succeed()
            result.resistance = resistance

        return result


def main(args=None):
    rclpy.init(args=args)

    can_mutex = Lock()
    bus = can.ThreadSafeBus(bustype='socketcan',
                            channel='can0', bitrate=500000)

    # Set filters for receiving data
    bus.set_filters(filters=[{
        'can_id': RESISANCE_METER_CAN_ID,
        'can_mask': RESISANCE_METER_CAN_MASK,
        'extended': True
    }])
    driver = ResistanceMeterDriver(can_mutex, bus)
    rclpy.spin(driver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
