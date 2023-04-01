import socket
import time
import random

serverAddressPort   = ("192.168.8.123", 20001)
bufferSize          = 1024


UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
while(True):
    msgFromClient = random.randint(0,20)
    bytesToSend = str.encode(str(msgFromClient))
    UDPClientSocket.sendto(bytesToSend, serverAddressPort)
    time.sleep(5)

    msgFromServer = UDPClientSocket.recvfrom(bufferSize)
    msg = "Message from Server: {}".format(msgFromServer[0])
    print(msg)
