#	-*- coding: utf-8 -*-
from __future__ import print_function
from zmq.eventloop import ioloop, zmqstream
import zmq

addr = 'tcp://127.0.0.1:45064'

def echo(msg):
    print(msg)
    
zmq_context = zmq.Context()
loop = ioloop.IOLoop.instance()
sock = zmq_context.socket(zmq.SUB)
sock.setsockopt(zmq.SUBSCRIBE, '')
sock.bind(addr)
_stream = zmqstream.ZMQStream(sock, loop)
_stream.on_recv(echo)
print(echo)
loop.start()