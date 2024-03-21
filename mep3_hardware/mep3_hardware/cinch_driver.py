#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import can
from std_msgs.msg import Int8


class CinchDriver(Node):

    def __init__(self):
        super().__init__('cinch_driver')
        self.__publisher = self.create_publisher(
            Int8, '/match_start_status', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.__can_bus = can.interface.Bus(channel='can0', bustype='socketcan')
    
    def wait_for_cinch_insertion(self):
        while True:
            message = self.__can_bus.recv()
            if message.arbitration_id == 0x6d00 and message.data[0] == 0:
                print("======== Chinc inserted! ========")
                return

    def wait_for_cinch_removal(self):
        while True:
            message = self.__can_bus.recv()
            if message.arbitration_id == 0x6d00 and message.data[0] == 1:
                # print("======== Chinc removed! ========")
                return

    def process_cinch_state(self):
        self.__publisher.publish(Int8(data=0))
        rclpy.spin_once(self, timeout_sec=0)

        # uncomment this if want to insert chinc after strating program
        # self.wait_for_cinch_insertion()
        # self.__publisher.publish(Int8(data=1))
        # rclpy.spin_once(self, timeout_sec=0)

        self.wait_for_cinch_removal()
        self.__publisher.publish(Int8(data=2))
        rclpy.spin_once(self, timeout_sec=0)


def main(args=None):
    rclpy.init(args=args)

    cinch_driver = CinchDriver()
    cinch_driver.process_cinch_state()

    cinch_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
