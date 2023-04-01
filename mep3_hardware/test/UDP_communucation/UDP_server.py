import socket

localIP     = "192.168.8.123"
localPort   = 20001
bufferSize  = 1024

msgFromServer       = "Sum from server: "
bytesToSend         = str.encode(msgFromServer)

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPServerSocket.bind((localIP, localPort))
print("UDP server up and listening")
sum = 0
while(True):
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    message = bytesAddressPair[0]
    address = bytesAddressPair[1]

    clientMsg = "Message from Client:{}".format(message)
    clientIP  = "Client IP Address:{}".format(address)
    received_points = clientMsg[22:-1]

    sum += int(received_points)
    
    print("Received points: ",received_points,", current sum: ", sum)
    print(clientIP)

    UDPServerSocket.sendto(bytesToSend, address)