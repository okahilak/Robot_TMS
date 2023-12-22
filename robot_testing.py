from socket import socket, AF_INET, SOCK_STREAM

IP = "192.168.200.251"
PORT = 10003
REQUEST_ENDING_CHARS = ",;"
RESPONSE_LENGTH = 1024
ROBOT_ID = 0

s = socket(AF_INET, SOCK_STREAM)
s.connect((IP, PORT))

request = "ReadRobotState,0,;"
s.sendall(request.encode('utf-8'))

response = s.recv(RESPONSE_LENGTH).decode('utf-8')
print(response)

request = "ReadActPos,0,;"
s.sendall(request.encode('utf-8'))

response = s.recv(RESPONSE_LENGTH).decode('utf-8')
print(response)

request = "MoveRelL,0,1,1,10,1,;"
s.sendall(request.encode('utf-8'))

response = s.recv(RESPONSE_LENGTH).decode('utf-8')
print(response)
