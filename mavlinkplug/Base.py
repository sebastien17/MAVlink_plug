#!/usr/bin/env python
#	-*- coding: utf-8 -*-

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#	This file is part of MAVlinkplug.

#	MAVlinkplug is free software: you can redistribute it and/or modify
#	it under the terms of the GNU General Public License as published by
#	the Free Software Foundation, either version 3 of the License, or
#	(at your option) any later version.

#	MAVlinkplug is distributed in the hope that it will be useful,
#	but WITHOUT ANY WARRANTY; without even the implied warranty of
#	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#	GNU General Public License for more details.

#	You should have received a copy of the GNU General Public License
#	along with MAVlinkplug.  If not, see <http://www.gnu.org/licenses/>.
#	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#Core Module
from __future__ import print_function

#External Module
from zmq.eventloop import ioloop, zmqstream
import zmq, multiprocessing, threading

#Internal Module
import mavlinkplug.Message


#Thread decorator
def in_thread(isDaemon = True):
    def base_in_thread(fn):
        '''
        Decorator to create a function in a new thread
        '''
        def wrapper(*args, **kwargs):
            t = threading.Thread(target=fn, args=args, kwargs=kwargs)
            t.setDaemon(isDaemon)
            t.start()
            return t
        return wrapper
    return base_in_thread

class MAVLinkPlugZmqBase(multiprocessing.Process):
    '''
    This is the base for all processes and offers utility functions
    for creating new streams.
    '''
    def __init__(self, zmq_context = None):
        super(MAVLinkPlugZmqBase,self).__init__()
        self._zmq_context =  zmq_context
        self._loop = None
        self.daemon = True
        self._ident = None
        self._default_subscribe = [mavlinkplug.Message.DEF_PACK(mavlinkplug.Message.MSG_PLUG_DEST_TYPE_ALL), mavlinkplug.Message.DEF_PACK(self._ident)]
    def setup(self):
        if(self._zmq_context == None):
            self._zmq_context = zmq.Context()
        self._loop = ioloop.IOLoop.instance()
    def stream(self, sock_type, addr, bind = True, callback=None, subscribe=None):
        if(subscribe == None):
            subscribe = self._default_subscribe
        sock = self._zmq_context.socket(sock_type)
        if (sock_type == zmq.SUB):
            for opt in subscribe:
                sock.setsockopt(zmq.SUBSCRIBE, opt)
        if (bind):
            sock.bind(addr)
        else:
            sock.connect(addr)
        _stream = zmqstream.ZMQStream(sock, self._loop)
        _stream.flush(zmq.POLLIN|zmq.POLLOUT)
        if (callback):
            _stream.on_recv(callback)
        return _stream
    def run(self):
        self.setup()
        self._loop.start()
    def stop(self):
        self._loop.stop()
    def insert_ident(self, string):
        return self._name + ' {0} : '.format(self._ident)

        
class MAVLinkPlugModBase(object):
    ''' Base class for all mods'''
    def __init__(self):
        self._run = False
        self._thread = []
        self.isDaemon = True
        self._ident = ''
        self._name = ''
    def _mod(self):
        pass
    def start(self):
        self._run = True
        @in_thread(self.isDaemon)
        def __t():
            self._mod()
        self._thread = __t()
    def stop(self):
        self._run = False
    def ident(self):
        '''Return connection ident'''
        return self._ident
    def info(self):
        '''Return connection information'''
        return {}
    def insert_ident(self, string):
        return self._name + ' {0} : '.format(self._ident)