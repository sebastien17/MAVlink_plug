#!/usr/bin/env python
#	-*- coding: utf-8 -*-

import zmq, time

_PORT = 'tcp://127.0.0.1:42568'

if(__name__ == '__main__'):
    _context = zmq.Context()
    socket = _context.socket(zmq.SUB)
    socket.connect(_PORT)
    socket.setsockopt(zmq.SUBSCRIBE, '')
    while(True):
        string = socket.recv()
        print(string)
