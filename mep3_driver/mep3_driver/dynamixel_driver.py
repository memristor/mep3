#!/usr/bin/env python3

# TODO: We should implement this with https://github.com/ros-controls/ros2_controllers/tree/master/joint_trajectory_controller
# once issues like https://github.com/ros-controls/ros2_controllers/issues/249 are resolved.

from functools import partial

from mep3_msgs.srv import DynamixelCommand
from rclpy.executors import MultiThreadedExecutor

import rclpy
from rclpy.node import Node

from threading import current_thread


MOTORS = [
    {'id': 1, 'name': 'shoulder_pan_joint'},
]


class DynamixelDriver(Node):
    def __init__(self):
        super().__init__('dynamixel_driver')
        self.__services = []
        for motor in MOTORS:
            self.__services.append(
                self.create_service(
                    DynamixelCommand,
                    f"dynamixel_command/{motor['name']}",
                    partial(self.__handle_dynamixel_command, motor_id=motor['id'])
                )
            )

    def __handle_dynamixel_command(self, request, response, motor_id=None):
        self.get_logger().info(str(request))
        self.get_logger().info(str(motor_id))
        self.get_logger().info(str(current_thread().name))

        response.result = 1
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
