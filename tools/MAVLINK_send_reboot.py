import zmq
from time import sleep

context = zmq.Context()
socket = context.socket(zmq.PUB)

socket.connect("tcp://localhost:42018")

sleep(2)

socket.send('CMD REBOOT')

socket.close()
context.destroy()