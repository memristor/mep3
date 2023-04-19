import socket
import select
import rclpy
import logging
from rclpy.node import Node
from mep3_msgs.msg import Scoreboard
from std_msgs.msg import Int8

UDP_IP = "192.168.8.129"  # IP address of the UDP server
UDP_PORT = 8888  # Port number of the UDP server
BUFFER_SIZE = 1024
MATCH_END_TIME = 85  # seconds


class ScoreboardSubscriber(Node):

    def __init__(self):
        super().__init__('scoreboard_subscriber')
        self.publisher = self.create_publisher(Scoreboard, '/scoreboard', 10)
        self.subscription = self.create_subscription(
            Int8,
            "/match_start_status",
            rclpy.qos.QoSProfile(reliability=rclpy.qos.QoSPolicyKind.RELIABLE,
                                durability=rclpy.qos.QoSPolicyKind.TRANSIENT_LOCAL),
            self.matchStartCallback)
        self.get_logger().info('Logging level: %d' % self.get_logger().get_effective_level())
        self.points = 0
        self.timer = 0
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

    def publish_scoreboard(self):
        self.timer += 1
        if self.timer < MATCH_END_TIME:
            self.sock.sendto(self.START_MESSAGE, (UDP_IP, UDP_PORT))
        else:
            END_MESSAGE = ("0202").encode('utf-8')
            self.sock.sendto(END_MESSAGE, (UDP_IP, UDP_PORT))

        ready, _, _ = select.select([self.sock], [], [], 0.1)  # Check if data is available to be read
        self.get_logger().info('Entered timer')
        if ready:
            data, addr = self.sock.recvfrom(BUFFER_SIZE)
            print("Received message:", data.decode())

            points = int(data)
            if points != self.points:
                scoreboard_msg = Scoreboard()
                scoreboard_msg.task = 'box'
                scoreboard_msg.points = points - self.points
                self.points = points
                self.publisher.publish(scoreboard_msg)
                self.get_logger().info('Published scoreboard: task=%s, points=%d' % (scoreboard_msg.task, scoreboard_msg.points))


def main(args=None):
    rclpy.init(args=args)
    node = ScoreboardSubscriber()
    rclpy.logging.set_logger_level('scoreboard_publisher', logging.DEBUG) # Set the logging level to DEBUG
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
