#!/usr/bin/env python3

from threading import Lock
import can
import struct
import curses

SERVOS = [
    {'id': 1, 'name': 'arm_left_motor_base', 'model': 'ax12'},
    {'id': 2, 'name': 'arm_left_motor_mid', 'model': 'mx28'},
    {'id': 3, 'name': 'arm_left_motor_gripper', 'model': 'ax12'},
    {'id': 4, 'name': 'arm_right_motor_base', 'model': 'ax12'},
    {'id': 5, 'name': 'arm_right_motor_mid', 'model': 'mx28'},
    {'id': 6, 'name': 'arm_right_motor_gripper', 'model': 'ax12'},
    {'id': 7, 'name': 'hand_mid_L', 'model': 'ax12'},
    {'id': 10, 'name': 'hand_right_Dz', 'model': 'ax12'},
    {'id': 9, 'name': 'hand_right_G', 'model': 'ax12'},
]

SERVO_CAN_ID = 0x00006C00
POLL_PERIOD = 0.2

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


class DynamixelDriver:
    def __init__(self, can_mutex, can_bus):
        self.servo_list = []
        self.can_mutex = can_mutex
        self.bus = can_bus
        for servo_config in SERVOS:
            new_servo = DynamixelServo(
                servo_id=servo_config['id'], name=servo_config['name'], model=servo_config['model']
            )
            self.servo_list.append(new_servo)
            #self.process_single_command(bin_data=new_servo.get_command_data('TorqueEnable', 0))
        self.stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()

    def process_single_command(self, bin_data):

        self.can_mutex.acquire()

        msg = can.Message(arbitration_id=SERVO_CAN_ID,
                          data=bin_data,
                          is_extended_id=True)

        try:
            self.bus.send(msg)
        except can.CanError:
            print("CAN ERROR: Cannot send message over CAN bus. Check if can0 is active.")

        message = self.bus.recv(0.1)  # Wait until a message is received or 0.1s

        self.can_mutex.release()

        if message:
            ret_val = message
        else:
            print("Timeout error for servo response. Check servo connections.")
            ret_val = False

        return ret_val

    def read(self):
      self.stdscr.clear()
      for i, servo in enumerate(self.servo_list):
        success = self.get_present_position(servo)
        position = None
        if success:
          position = servo.increment_to_degree(servo.present_position)

        servo_name = servo.name.ljust(40)
        self.stdscr.addstr(i, 0, f'{servo_name} {position}')
      self.stdscr.refresh()

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
                print("Wrong response")
                ret_val = 0
        return ret_val

    def __del__(self):
      curses.echo()
      curses.nocbreak()
      curses.endwin()


def main(args=None):
    can_mutex = Lock()
    bus = can.ThreadSafeBus(bustype='socketcan', channel='can0', bitrate=500000)
    bus.set_filters(filters=[{"can_id": SERVO_CAN_ID, "can_mask": 0x1FFFFFFF, "extended": True}])
    driver = DynamixelDriver(can_mutex, bus)

    while True:
        driver.read()

if __name__ == '__main__':
    main()
