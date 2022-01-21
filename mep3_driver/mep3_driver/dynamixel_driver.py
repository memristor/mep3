#!/usr/bin/env python3

# TODO: We should implement this with https://github.com/ros-controls/ros2_controllers/tree/master/joint_trajectory_controller
# once issues like https://github.com/ros-controls/ros2_controllers/issues/249 are resolved.

from distutils import command
from functools import partial

from mep3_msgs.srv import DynamixelCommand
from rclpy.executors import MultiThreadedExecutor

import rclpy
from rclpy.node import Node
from time import sleep

from threading import current_thread, Lock

import can
import struct
from math import isclose

SERVOS = [
    {'id': 3, 'name': 'servo3', 'model': 'ax12'},
    {'id': 1, 'name': 'servo1', 'model': 'ax12'},
]

SERVO_CAN_ID = 0x00006C00
POOL_PERIOD = 0.2

servo_commands = {
    'ModelNumber': [0, 'R', 'h'],

    'SetId': [3, 'RW', 'B'],
    'SetBaud': [4, 'RW', 'B'],
    'ReturnDelayTime': [5, 'RW', 'B'],
    'CWAngleLimit': [6, 'RW', 'h'],
    'CCWAngleLimit': [8, 'RW', 'h'],
    'HLimitTemp': [11, 'RW', 'B'],
    'MinVoltage': [12, 'RW', 'B'],
    'MaxVoltage': [13, 'RW', 'B'],
    'MaxTorque': [14, 'RW', 'h'],
    'Status': [16, 'RW', 'B'],
    'AlarmLED': [17, 'RW', 'B'],
    'AlarmShutdown': [18, 'RW', 'B'],
    'TorqueEnable': [24, 'RW', 'B'],
    'LED': [25, 'RW', 'B'],
    'CWComplianceMargin': [26, 'RW', 'B'],
    'CCWComplianceMargin': [27, 'RW', 'B'],
    'CWComplianceScope': [28, 'RW', 'B'],
    'CCWComplianceScope': [29, 'RW', 'B'],
    'GoalPosition': [30, 'RW', 'h'],
    'Speed': [32, 'RW', 'h'],
    'TorqueLimit': [34, 'RW', 'B'],
    'PresentPosition': [36, 'R', 'h'],
    'PresentSpeed': [38, 'R', 'h'],
    'PresentLoad': [40, 'R', 'h'],
    'PresentVoltage': [42, 'R', 'B'],
    'PresentTemp': [43, 'R', 'B'],
    'Punch': [48, 'RW', 'h'],
}


class DynamixelServo:
    def __init__(self, servo_id, name, model):
        self.id = servo_id
        self.name = name
        self.model = model

        # Values we need to keep track of while Node is active
        self.present_position = None
        self.present_velocity = None

    def get_command_data(self, command, val):
        # doing only GoalPosition action for now and only AX12

        if self.model != 'ax12':
            print("Only AX12 for now!")
            return False
        cmd = servo_commands[command]

        val = int(val) if val else None
        servo_len = 4
        servo_func = cmd[0]
        servo_rw = cmd[1]
        pfmt = servo_fmt = cmd[2]

        if (val is None) and ('R' not in servo_rw):
            print('function ' + command + ' is not readable')
            return

        if (val is not None) and ('W' not in servo_rw):
            print('function ' + command + ' is not writable')
            return

        if val is None:
            servo_rw = 2
            servo_fmt = 'B'
            servo_len = 4
        else:
            servo_rw = 3
            if servo_fmt == 'h':
                servo_len += 1

        fmt = '4B' + servo_fmt
        data = [self.id, servo_len, servo_rw, servo_func]

        if val is not None:
            data += [val]
        else:
            data += [2] if pfmt == 'h' else [1]

        binary_data = struct.pack(fmt, *data)

        return binary_data


class DynamixelDriver(Node):
    def __init__(self, servo, can_mutex):
        super().__init__('dynamixel_driver' + str(servo['id']))
        self.__services = []
        self.servo_list = []

        self.can_mutex = can_mutex

        self.rate = self.create_rate(1 / POOL_PERIOD)

        new_servo = DynamixelServo(
            servo_id=servo['id'], name=servo['name'], model=servo['model']
        )
        self.servo_list.append(new_servo)
        self.__services.append(
            self.create_service(
                DynamixelCommand,
                f"dynamixel_command/{servo['name']}",
                partial(self.__handle_dynamixel_command, servo=new_servo)
            )
        )

    def process_single_command(self, bin_data):

        self.can_mutex.acquire()
        bus = can.ThreadSafeBus(bustype='socketcan', channel='can0', bitrate=500000)
        # Set filters for receiving data
        bus.set_filters(filters=[{"can_id": SERVO_CAN_ID, "can_mask": 0xFFFF, "extended": True}])

        msg = can.Message(arbitration_id=SERVO_CAN_ID,
                          data=bin_data,
                          is_extended_id=True)

        try:
            bus.send(msg)
        except can.CanError:
            self.get_logger().info("CAN ERROR: Nije poslata poruka")

        message = bus.recv(0.2)  # Wait until a message is received or 1s

        self.can_mutex.release()

        if message:
            ret_val = message
        else:
            self.get_logger().info("Istekao je timeout za statusnu poruku")
            ret_val = False

        return ret_val

    def __handle_dynamixel_command(self, request, response,
                                   servo: DynamixelServo = None):
        self.get_logger().info(str(request))
        self.get_logger().info("Servo ID: " + str(servo.id))
        self.get_logger().info("Thread: " + str(current_thread().name))

        response.result = 0

        if not servo.present_position:
            # Need to ask servo for position
            if not self.get_present_position(servo):
                return response

        if not servo.present_velocity:
            # Need to ask servo for speed
            if not self.get_present_velocity(servo):
                return response

        if request.velocity != servo.present_velocity:
            # Setting servo speed, if necessary
            if not self.set_velocity(servo, request.velocity):
                return response

        if not isclose(request.position, servo.present_position,
                       abs_tol=request.tolerance):
            # Send GoalPosition and Pool
            if not self.go_to_position(servo, request.position,
                                       request.timeout, request.tolerance):
                return response

        response.result = 1
        return response

    def get_present_position(self, servo):
        ret_val = 1
        status = self.process_single_command(
            bin_data=servo.get_command_data('PresentPosition', None))

        if not status:
            ret_val = 0
        else:
            if (len(status.data) == 5):
                servo.present_position = float(struct.unpack(
                    servo_commands['PresentPosition'][2], status.data[3:])[0])
            else:
                self.get_logger().info("Wrong response")
                ret_val = 0
        return ret_val

    def get_present_velocity(self, servo):
        ret_val = 1
        status = self.process_single_command(
            bin_data=servo.get_command_data('Speed', None))

        if not status:
            ret_val = 0
        else:
            if (len(status.data) == 5):
                servo.present_velocity = float(struct.unpack(
                    servo_commands['Speed'][2], status.data[3:])[0])
            else:
                self.get_logger().info("Wrong response")
                ret_val = 0
        return ret_val

    def set_velocity(self, servo, velocity):
        ret_val = 1
        status = self.process_single_command(
            bin_data=servo.get_command_data('Speed', velocity))

        if not status:
            ret_val = 0
        elif status.data[2] != 0x00:
            ret_val = 0

        return ret_val

    def go_to_position(self, servo, position, timeout, tolerance):
        status = self.process_single_command(
            bin_data=servo.get_command_data('GoalPosition', position))
        if not status:
            return 0

        number_of_tries = 0

        while not isclose(position, servo.present_position,
                          abs_tol=tolerance):

            self.rate.sleep()
            self.get_present_position(servo)

            if number_of_tries > (timeout / POOL_PERIOD):
                return 0

            number_of_tries += 1

        return 1


def main(args=None):
    rclpy.init(args=args)

    can_mutex = Lock()

    executor = MultiThreadedExecutor(num_threads=6)
    for servo in SERVOS:
        driver = DynamixelDriver(servo, can_mutex)
        executor.add_node(driver)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
