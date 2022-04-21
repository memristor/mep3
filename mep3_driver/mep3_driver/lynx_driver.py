#!/usr/bin/env python3

from distutils import command
from functools import partial
from traceback import print_tb

from mep3_msgs.action import DynamixelCommand
from rclpy.executors import MultiThreadedExecutor

import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup


DEFAULT_POSITION = 0  # deg
MAX_POSITION = 155    # deg
MIN_POSITION = 0    # deg
DEFAULT_VELOCITY = 10  # deg/s
DEFAULT_TOLERANCE = 5  # deg
DEFAULT_TIMEOUT = 5  # s
"""
Test:
ros2 action send_goal /big/dynamixel_command/lift_motor mep3_msgs/action/DynamixelCommand "position: 22"
"""

from threading import current_thread, Lock

import can
import struct
from math import isclose

SERVOS = [
    {'id': 0, 'name': 'lift_motor', 'model': 'lynx'},
]

SERVO_CAN_ID = 0x00006C20
POLL_PERIOD = 0.2


def scale(val, src, dst):
    return ((val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]


class LynxServo:
    def __init__(self, servo_id, name, model):
        self.id = servo_id
        self.name = name
        self.model = model

        # Values we need to keep track of while Node is active
        self.present_position = 0
        self.present_velocity = 0

    def get_command_data(self, position):

        if position > MAX_POSITION: position = MAX_POSITION
        if position < MIN_POSITION: position = MIN_POSITION
        position = int(position*10) # Scaling to Lynx protocol

        fmt = '>BHH'
        speed = int(37*10)		    # Scaling to Lynx protocol

        data = [self.id, position, speed]

        binary_data = struct.pack(fmt, *data)

        return binary_data


class LynxDriver(Node):
    def __init__(self, can_mutex, can_bus):
        super().__init__('lynx_driver')
        self.__actions = []
        self.servo_list = []

        self.can_mutex = can_mutex
        self.bus = can_bus

        self.rate = self.create_rate(1 / POLL_PERIOD)

        callback_group = ReentrantCallbackGroup()
        for servo_config in SERVOS:
            new_servo = LynxServo(
                servo_id=servo_config['id'], name=servo_config['name'], model=servo_config['model']
            )
            self.servo_list.append(new_servo)
            self.__actions.append(
                ActionServer(
                    self,
                    DynamixelCommand,
                    f"dynamixel_command/{servo_config['name']}",
                    execute_callback=partial(self.__handle_dynamixel_command, servo=new_servo),
                    callback_group=callback_group,
                    goal_callback=self.__goal_callback,
                    cancel_callback=self.__cancel_callback
                )
            )

    def __goal_callback(self, _):
        return GoalResponse.ACCEPT

    def __cancel_callback(self, _):
        return CancelResponse.ACCEPT

    def process_single_command(self, bin_data):

        self.can_mutex.acquire()

        msg = can.Message(arbitration_id=SERVO_CAN_ID,
                          data=bin_data,
                          is_extended_id=True)

        try:
            self.bus.send(msg)
        except can.CanError:
            self.get_logger().info("CAN ERROR: Cannot send message over CAN bus. Check if can0 is active.")

        message = self.bus.recv(0.1)  # Wait until a message is received or 0.1s

        self.can_mutex.release()

        if message:
            ret_val = message
        else:
            self.get_logger().info("Timeout error for servo response. Check servo connections.")
            ret_val = False

        return ret_val

    def __handle_dynamixel_command(self, goal_handle,
                                   servo: LynxServo = None):
        self.get_logger().info(str(goal_handle.request))
        self.get_logger().info("Servo ID: " + str(servo.id))
        self.get_logger().info("Thread: " + str(current_thread().name))

        position = goal_handle.request.position  # deg
        position += 6 # For some reason Servo doesn't go to 5 or lower

        if not position:
            position = 0.1

        tolerance = goal_handle.request.tolerance  # deg
        timeout = goal_handle.request.timeout

        if not tolerance:
            tolerance = DEFAULT_TOLERANCE
        if not timeout:
            timeout = DEFAULT_TIMEOUT

        result = DynamixelCommand.Result()

        result.result = 2  # Other error

        if not isclose(position, servo.present_position,
                       abs_tol=tolerance):
            # Send GoalPosition and Poll
            if not self.go_to_position(servo, position,
                                       timeout, tolerance):
                result.result = 1  # Set to Timeout error
                self.get_logger().info("Timeout!!!")

                goal_handle.abort()
                return result

        result.result = 0  # There are no errors, return 0
        goal_handle.succeed()
        return result


    def go_to_position(self, servo, position, timeout, tolerance):
        status = self.process_single_command(
            bin_data=servo.get_command_data(position))
        if not status:
            return 0

        number_of_tries = 0

        while not isclose(position, servo.present_position,
                          abs_tol=tolerance):

            self.rate.sleep()
            status = self.process_single_command(
                    bin_data=servo.get_command_data(position))
            
            if status:

                fmt = '>BHH'
                if len(status.data) == 5:
                    servo.present_position = float( struct.unpack(fmt, status.data)[1])/10
                    servo.present_velocity = float( struct.unpack(fmt, status.data)[2])/10
                self.get_logger().info("Servo responded pos: {}".format(servo.present_position))
            else:
                self.get_logger().info("Servo response is not OK")
				

            if number_of_tries > (timeout / POLL_PERIOD):
                return 0

            number_of_tries += 1

        return 1


def main(args=None):
    rclpy.init(args=args)

    can_mutex = Lock()
    bus = can.ThreadSafeBus(bustype='socketcan', channel='can0', bitrate=500000)

    # Set filters for receiving data
    bus.set_filters(filters=[{"can_id": SERVO_CAN_ID, "can_mask": 0x1FFFFFFE, "extended": True}])

    executor = MultiThreadedExecutor(num_threads=6)
    driver = LynxDriver(can_mutex, bus)
    executor.add_node(driver)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
