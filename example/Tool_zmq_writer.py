#!/usr/bin/env python
#	-*- coding: utf-8 -*-

import zmq, time

_PORT = 'tcp://127.0.0.1:42569'

if(__name__ == '__main__'):
    _context = zmq.Context()
    socket = _context.socket(zmq.PUB)
    socket.connect(_PORT)
    num = 0
    while(True):
        string = socket.send(str(num))
        print(num)
        num = num + 1
