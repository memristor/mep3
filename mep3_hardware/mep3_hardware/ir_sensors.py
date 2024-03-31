#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import can
from can import Message

filters = [
{"can_id": 0x6d10, "can_mask": 0x1FFFFFFF, "extended": True},
{"can_id": 0x6d11, "can_mask": 0x1FFFFFFF, "extended": True},
{"can_id": 0x6d12, "can_mask": 0x1FFFFFFF, "extended": True},
{"can_id": 0x6d13, "can_mask": 0x1FFFFFFF, "extended": True},
]

class IRSensorPublisher(Node):
    def __init__(self):
        super().__init__('ir_sensor_publisher')
        self._publisher = self.create_publisher(Int32, 'ir_sensor_state', 10)
        self._timer = self.create_timer(0.005, self.publish_ir_sensor_state)
        self._can_bus = can.interface.Bus(channel="can0", interface="socketcan", can_filters=filters)

    def publish_ir_sensor_state(self):
        message = self._can_bus.recv()
        
        msg = Int32()
        msg.data = message.data[0]
        self._publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    ir_sensor_publisher = IRSensorPublisher()
    rclpy.spin(ir_sensor_publisher)
    ir_sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
