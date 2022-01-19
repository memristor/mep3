#!/usr/bin/env python3

# TODO: We should implement this with https://github.com/ros-controls/ros2_controllers/tree/master/joint_trajectory_controller
# once issues like https://github.com/ros-controls/ros2_controllers/issues/249 are resolved.

from functools import partial

from mep3_msgs.srv import DynamixelCommand
from rclpy.executors import MultiThreadedExecutor

import rclpy
from rclpy.node import Node

from threading import current_thread

import socket
import struct
from math import isclose

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

SERVOS = [
    {'id': 1, 'name': 'shoulder_pan_joint', 'model': 'ax12'},
]


class DynamixelServo:
    def __init__(self, servo_id, name, model):
        self.id = servo_id
        self.name = name
        self.model = model

        # Values we need to keep track of while Node is active
        self.old_position = 5
        self.old_velocity = 5

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

        print("upakovano: " + str(data) +
              "\nformat: " + str(fmt) +
              "\nbinarno: " + str(binary_data))

        return binary_data


class DynamixelDriver(Node):
    def __init__(self):
        super().__init__('dynamixel_driver')
        self.__services = []
        self.servo_list = []
        for servo in SERVOS:
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

        self.ps = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ps.connect(('127.0.0.1', 12345))

    def process_single_command(self, bin_data):
        ret_val = True

        self.get_logger().info('sending data')
        self.ps.sendall(bin_data)

        # Wait for response
        self.ps.settimeout(0.1)
        try:
            response = self.ps.recv(2048)
            print("Stigao odgovor")
            if response:
                self.get_logger().info('response' + str(struct.unpack('4Bh', response)))
                ret_val = struct.unpack('4Bh', response)
        except:
            self.get_logger().info("Status Message Timeout")
            ret_val = False

        # Status message
        return ret_val

    def __handle_dynamixel_command(self, request, response,
                                   servo: DynamixelServo = None):
        self.get_logger().info(str(request))
        self.get_logger().info("Servo ID: " + str(servo.id))
        self.get_logger().info("Thread: " + str(current_thread().name))

        response.result = 1

        if not servo.old_position:
            # Need to ask servo for position
            status = self.process_single_command(
                bin_data=servo.get_command_data('PresentPosition', None))
            if not status:
                return False

        if not servo.old_velocity:
            # Need to ask servo for speed
            status = self.process_single_command(
                bin_data=servo.get_command_data('PresentSpeed', None))

        if not isclose(request.position, servo.old_position,
                       abs_tol=request.tolerance):
            # Send GoalPosition and Pool
            status = self.process_single_command(
                bin_data=servo.get_command_data('GoalPosition', request.position))
            if not status:
                response.result = 0

            # TODO: Pool every 100ms to check if the servo finished movement
            #number_of_tries = int(request.timeout,

        return response


def main(args=None):
    rclpy.init(args=args)

    driver = DynamixelDriver()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(driver)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
