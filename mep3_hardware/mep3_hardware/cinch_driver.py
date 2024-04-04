#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from mep3_msgs.msg import Scoreboard
import can
from std_msgs.msg import Int8
import socket
import select, time

UDP_IP_1 = "192.168.8.119"  
UDP_IP_2 = "192.168.8.116"  
UDP_IP_3 = "192.168.8.118"

UDP_PORT = 8888
BUFFER_SIZE = 1024 

filters = [
{"can_id": 0x6d00, "can_mask": 0x1FFFFFFF, "extended": True}
]

class CinchDriver(Node):

    def __init__(self):
        super().__init__('cinch_driver')
        self.__publisher = self.create_publisher(
            Int8, '/match_start_status', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.__can_bus = can.interface.Bus(channel='can0', bustype='socketcan', can_filters=filters)

        # self.lcd_publisher = self.create_publisher(Scoreboard, '/scoreboard', 10)

        self.ip_addreses = [UDP_IP_1, UDP_IP_2, UDP_IP_3]
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.start_msg = "1212"
        self.start_time = None
        # self.is_first_came = False
        # self.is_second_came = False
        # self.is_third_came = False
        self.number_of_send_msgs = 0
    
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
                self.start_time = time.time()
                return

    def process_cinch_state(self):
        self.__publisher.publish(Int8(data=0))
        # rclpy.spin_once(self, timeout_sec=0)

        # uncomment this if want to insert chinc after strating program
        # self.wait_for_cinch_insertion()
        # self.__publisher.publish(Int8(data=1))
        # rclpy.spin_once(self, timeout_sec=0)

        self.wait_for_cinch_removal()
        self.__publisher.publish(Int8(data=2))
        print(time.time() - self.start_time)
        
        if self.start_time is not None:
                while time.time() - self.start_time < 90:
                    MESSAGE = ("3333").encode('utf-8') 
                    for ip_address in self.ip_addreses:
                        self.sock.sendto(MESSAGE, (ip_address, UDP_PORT))

                MESSAGE = (self.start_msg).encode('utf-8')
                while time.time() - self.start_time < 96:
                    for ip_address in self.ip_addreses:
                        self.sock.sendto(MESSAGE, (ip_address, UDP_PORT))

                        print(f"Sent message to {ip_address}:", MESSAGE.decode())
                        self.number_of_send_msgs+=1

                
                
            
                # while time.time() - self.start_time < 91:
                #     data, addr = self.sock.recvfrom(BUFFER_SIZE)
                #     print(f"Received message from ip_address {ip_address}:", data.decode())
                #     if int(data.decode())==1 and  not self.is_first_came:
                #         scoreboard_msg = Scoreboard()
                #         scoreboard_msg.points = 15
                #         scoreboard_msg.task = 'SIMA'+str(ip_address)
                #         self.is_first_came  = True
                #         self.lcd_publisher.publish(scoreboard_msg)
                    
                #     if int(data.decode())==2 and  not self.is_second_came:
                #         scoreboard_msg = Scoreboard()
                #         scoreboard_msg.points = 10
                #         scoreboard_msg.task = 'SIMA'+str(ip_address)
                #         self.is_second_came  = True
                #         self.lcd_publisher.publish(scoreboard_msg)
                    
                #     if int(data.decode())==3 and  not self.is_third_came:
                #         scoreboard_msg = Scoreboard()
                #         scoreboard_msg.points = 10
                #         scoreboard_msg.task = 'SIMA'+str(ip_address)
                #         self.is_third_came  = True
                #         self.lcd_publisher.publish(scoreboard_msg)
        rclpy.spin_once(self, timeout_sec=0)


def main(args=None):
    rclpy.init(args=args)

    cinch_driver = CinchDriver()
    cinch_driver.process_cinch_state()

    cinch_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
