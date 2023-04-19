import socket
import select

UDP_IP = "192.168.8.129"  # IP address of the UDP server (change this to the IP of your ESP32)
UDP_PORT = 8888  # Port number of the UDP server (change this to the port number you used in your ESP32 code)
BUFFER_SIZE = 1024 

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
counter = 0
while (True):
    MESSAGE = ("Hello, server! " + str(counter)).encode('utf-8')  # The message you want to send to the server

    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

    print("Sent message:", MESSAGE.decode())
    counter += 1

    ready, _, _ = select.select([sock], [], [], 0.1)  # Check if data is available to be read
    if ready:
        data, addr = sock.recvfrom(BUFFER_SIZE)
        print("Received message:", data.decode())


sock.close()
