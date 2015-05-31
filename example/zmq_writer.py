#!/usr/bin/env python
#	-*- coding: utf-8 -*-

import zmq, time

_PORT = 'tcp://127.0.0.1:45064'

if(__name__ == '__main__'):
    _context = zmq.Context()
    socket = _context.socket(zmq.PUB)
    socket.connect(_PORT)
    while(True):
        string = socket.send('Youpi')
        print('Youpi')
        time.sleep(1)
