#!/usr/bin/env python3

# TODO: We should implement this with https://github.com/ros-controls/ros2_controllers/tree/master/joint_trajectory_controller
# once issues like https://github.com/ros-controls/ros2_controllers/issues/249 are resolved.

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
DEFAULT_VELOCITY = 45  # deg/s
DEFAULT_TOLERANCE = 1  # deg
DEFAULT_TIMEOUT = 5  # s
"""
Test:
ros2 action send_goal /big/dynamixel_command/arm_right_motor_base mep3_msgs/action/DynamixelCommand "position: 22"
"""

from threading import current_thread, Lock

import can
import struct
from math import isclose

SERVOS = [
    {'id': 1, 'name': 'arm_left_motor_base', 'model': 'ax12'},
    {'id': 2, 'name': 'arm_left_motor_mid', 'model': 'mx28'},
    {'id': 3, 'name': 'arm_left_motor_gripper', 'model': 'ax12'},
    {'id': 4, 'name': 'arm_right_motor_base', 'model': 'ax12'},
    {'id': 5, 'name': 'arm_right_motor_mid', 'model': 'mx28'},
    {'id': 6, 'name': 'arm_right_motor_gripper', 'model': 'ax12'},
    {'id': 9, 'name': 'hand_right_G', 'model': 'ax12'},
    {'id': 10,'name': 'hand_right_Dz', 'model': 'ax12'},
    {'id': 7, 'name': 'hand_mid_L', 'model': 'ax12'},
    {'id': 69,'name': 'hand_mid_S', 'model': 'ax12'},
    {'id': 55,'name': 'flipper_right', 'model': 'ax12'},
    {'id': 55,'name': 'flipper_left', 'model': 'ax12'},
]

SERVO_CAN_ID = 0x00006C00
POLL_PERIOD = 0.4

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
    'MovingSpeed': [32, 'RW', 'h'],
    'TorqueLimit': [34, 'RW', 'B'],
    'PresentPosition': [36, 'R', 'h'],
    'PresentSpeed': [38, 'R', 'h'],
    'PresentLoad': [40, 'R', 'h'],
    'PresentVoltage': [42, 'R', 'B'],
    'PresentTemp': [43, 'R', 'B'],
    'Punch': [48, 'RW', 'h'],
}


def scale(val, src, dst):
    return ((val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]


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

    def get_servo_velocity(self, degree_per_sec):
        """ Minimal difference for ax12 and mx28 (114 or 116 rpm max)"""
        # Max velocity is 696 degree/sec
        return scale(degree_per_sec, [0, 696], [0, 1023])

    def increment_per_degree(self, degree):
        increment = 0

        if self.model == "ax12":
            increment = scale(degree, [0, 300], [0, 1023])
        else:
            increment = scale(degree, [0, 360], [0, 4095])

        return increment

    def degree_to_increment(self, degree):
        increment = 0

        if self.model == "ax12":
            increment = scale(degree, [-150, 150], [0, 1023])
        else:
            increment = scale(degree, [-180, 180], [0, 4095])

        return increment

    def increment_to_degree(self, increment):
        degree = 0

        if self.model == "ax12":
            degree = scale(increment, [0, 1023], [-150, 150])
        else:
            degree = scale(increment, [0, 4095], [-180, 180])

        return degree


class DynamixelDriver(Node):
    def __init__(self, can_mutex, can_bus):
        super().__init__('dynamixel_driver')
        self.__actions = []
        self.servo_list = []

        self.can_mutex = can_mutex
        self.bus = can_bus

        self.rate = self.create_rate(1 / POLL_PERIOD)

        callback_group = ReentrantCallbackGroup()
        for servo_config in SERVOS:
            new_servo = DynamixelServo(
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
                                   servo: DynamixelServo = None):
        self.get_logger().info(str(goal_handle.request))
        self.get_logger().info("Servo ID: " + str(servo.id))
        self.get_logger().info("Thread: " + str(current_thread().name))

        position = goal_handle.request.position  # deg
        if not position:
            position = 0.1
        position_increment = servo.degree_to_increment(position)  # inc
        velocity = servo.get_servo_velocity(goal_handle.request.velocity)  # deg/s to servo velocity
        tolerance = servo.increment_per_degree(goal_handle.request.tolerance)  # inc
        timeout = goal_handle.request.timeout

        if not tolerance:
            tolerance = servo.increment_per_degree(DEFAULT_TOLERANCE)
        if not timeout:
            timeout = DEFAULT_TIMEOUT
        if not velocity:
            velocity = servo.get_servo_velocity(DEFAULT_VELOCITY)

        result = DynamixelCommand.Result()

        result.result = 0  # hardcode result

        if not servo.present_position:
            # Need to ask servo for position
            if not self.get_present_position(servo):
                goal_handle.abort()
                return result

        if not servo.present_velocity:
            # Need to ask servo for speed
            if not self.get_present_velocity(servo):
                goal_handle.abort()
                return result

        if velocity != servo.present_velocity:
            # Setting servo speed, if necessary
            if not self.set_velocity(servo, velocity):
                goal_handle.abort()
                return result

        if not isclose(position_increment, servo.present_position,
                       abs_tol=tolerance):
            # Send GoalPosition and Poll
            if not self.go_to_position(servo, position_increment,
                                       timeout, tolerance):
                result.result = 1  # Set to Timeout error
                self.get_logger().info("Timeout!!!")

                goal_handle.abort()
                return result

        result.result = 0  # There are no errors, return 0
        goal_handle.succeed()
        return result

    def get_present_position(self, servo):
        ret_val = 1
        status = self.process_single_command(
            bin_data=servo.get_command_data('PresentPosition', None))

        if not status:
            ret_val = 0
        else:
            if len(status.data) == 5:
                servo.present_position = float(struct.unpack(
                    servo_commands['PresentPosition'][2], status.data[3:])[0])
            else:
                self.get_logger().info("Wrong response, present position")
                self.get_logger().info(str(status.data))


                ret_val = 0
        return ret_val

    def get_present_velocity(self, servo):
        ret_val = 1
        status = self.process_single_command(
            bin_data=servo.get_command_data('MovingSpeed', None))

        if not status:
            ret_val = 0
        else:
            if len(status.data) == 5:
                servo.present_velocity = float(struct.unpack(
                    servo_commands['MovingSpeed'][2], status.data[3:])[0])
            else:
                self.get_logger().info("Wrong response, present velocity")
                self.get_logger().info(str(status.data))
                ret_val = 1 # hardcode success
        return ret_val

    def set_velocity(self, servo, velocity):
        ret_val = 1
        status = self.process_single_command(
            bin_data=servo.get_command_data('MovingSpeed', velocity))

        if not status:
            ret_val = 0
        elif status.data[2] != 0x00:
            ret_val = 0

        return ret_val

    def go_to_position(self, servo, position, timeout, tolerance):
        status = self.process_single_command(
            bin_data=servo.get_command_data('GoalPosition', position))
        if not status:
            self.get_logger().info("Wrong response")
            return 1 # hardcode success

        number_of_tries = 0

        while not isclose(position, servo.present_position,
                          abs_tol=tolerance):

            self.rate.sleep()
            self.get_present_position(servo)

            if number_of_tries > (timeout / POLL_PERIOD):
                return 1 #hardcode success

            number_of_tries += 1

        return 1


def main(args=None):
    rclpy.init(args=args)

    can_mutex = Lock()
    bus = can.ThreadSafeBus(bustype='socketcan', channel='can0', bitrate=500000)

    # Set filters for receiving data
    bus.set_filters(filters=[{"can_id": SERVO_CAN_ID, "can_mask": 0x1FFFFFFF, "extended": True}])

    executor = MultiThreadedExecutor(num_threads=6)
    driver = DynamixelDriver(can_mutex, bus)
    executor.add_node(driver)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
