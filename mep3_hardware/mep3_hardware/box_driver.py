#!/usr/bin/env python3

import socket
import select
import rclpy
import logging
from rclpy.node import Node
from mep3_msgs.msg import Scoreboard
from std_msgs.msg import Int8
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

UDP_IP = "192.168.8.129"  # IP address of the UDP server (hopefully!!!)
# UDP_IP = "192.168.8.101"  # IP address of Vincan's laptop
UDP_PORT = 8888  # Port number of the UDP server
BUFFER_SIZE = 1024


class ScoreboardSubscriber(Node):

    def __init__(self):
        super().__init__('scoreboard_subscriber')
        self.publisher = self.create_publisher(Scoreboard, '/scoreboard', 10)
        self.subscription = self.create_subscription(
            Int8,
            '/match_start_status',
            self.match_start_callback,
            qos_profile=QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.get_logger().info('Logging level: %d' % self.get_logger().get_effective_level())
        self.points = 0
        self.balls_in_box = False
        self.START_MESSAGE = ("0202").encode('utf-8')

        # Create a UDP socket and listen for incoming data
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.sock.bind((UDP_IP, UDP_PORT))
        # self.sock.setblocking(False)

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_scoreboard)

    def match_start_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if msg.data == 2:
            self.START_MESSAGE = ("1212").encode('utf-8')
            self.state = 1

    def publish_scoreboard(self):
        self.sock.sendto(self.START_MESSAGE, (UDP_IP, UDP_PORT))
        self.get_logger().info(self.START_MESSAGE)

        ready, _, _ = select.select([self.sock], [], [], 0.1)  # Check if data is available to be read
        if ready:
            data, addr = self.sock.recvfrom(BUFFER_SIZE)
            self.get_logger().info("Received message:" + data.decode())

            points = int(data)
            if points != self.points:
                scoreboard_msg = Scoreboard()
                scoreboard_msg.task = 'box'
                scoreboard_msg.points = points - self.points
                self.points = points
                self.publisher.publish(scoreboard_msg)
                self.get_logger().info('Published scoreboard: task=%s, points=%d' % (scoreboard_msg.task, scoreboard_msg.points))
                if not self.balls_in_box:
                    self.balls_in_box = True
                    scoreboard_msg = Scoreboard()
                    scoreboard_msg.task = 'box_correct_prediction_5_points'
                    scoreboard_msg.points = 5
                    self.publisher.publish(scoreboard_msg)



def main(args=None):
    rclpy.init(args=args)
    node = ScoreboardSubscriber()
    rclpy.logging.set_logger_level('scoreboard_publisher', logging.DEBUG) # Set the logging level to DEBUG
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
