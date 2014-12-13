import socket

address = ('127.0.0.1',22222)
r_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #TCP
r_socket.connect(address)

while(True):
    data = r_socket.recv(4096)
    print '*' + data