#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

#Import
import zmq
import logging
import threading
import socket
from pymavlink import mavutil
from time import sleep, time
from json import dumps, loads

ZMQ_MESSAGE_BINARY = 'B'
ZMQ_MESSAGE_JSON   = 'C'

#Threading decorator definition
def in_thread(isDaemon = True):
    def base_in_thread(fn):
        '''Decorator to create a threaded function '''
        def wrapper(*args, **kwargs):
            t = threading.Thread(target=fn, args=args, kwargs=kwargs)
            t.setDaemon(isDaemon)
            t.start()
            return t
        return wrapper
    return base_in_thread

def _print(_string):
    print(_string)
    logging.info(_string)

class ModBase(object):
    ''' Base class for all mods'''
    def __init__(self):
        self._run = False
        self._thread = None
        self.isDaemon = True
        self._ident = ''
    def _mod(self):
        pass
    def run(self):
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
    
class Bridge(ModBase):
        def __init__(self, zmq_context):
            super(Bridge, self).__init__()
            self._zmq_context = zmq_context
        def _mod(self):
            '''Create the ZMQ bridge between in & out'''
            socket_in =  self._zmq_context.socket(zmq.SUB)
            socket_in.bind('inproc://in')                           #Bind because most stable
            socket_in.setsockopt(zmq.SUBSCRIBE, '')
            socket_out =  self._zmq_context.socket(zmq.PUB)
            socket_out.bind('inproc://out')                         #Bind because most stable
            while (self._run):
                string = socket_in.recv()                           #Yield to event loop
                socket_out.send(string)    
            socket.close()
            
class MAVLINK_connection(ModBase):
        def __init__(self, zmq_context, ident, *argv, **kwargs):
            super(MAVLINK_connection, self).__init__()
            self._zmq_context = zmq_context
            self.isDaemon = False
            self._argv = argv
            self._kwargs = kwargs
            self._ident = ident
            self._mavh = None
            self._in_msg = 0
            self._ok_msg = 0
            self._out_msg = 0
        def try_connection(self):
            self._mavh = None
            _print('MAVLINK connection {0} initialising'.format(self._ident))
            while(self._run):
                try:
                    self._mavh = mavutil.mavlink_connection(*self._argv,**self._kwargs)
                except:
                    self._mavh = None
                    sleep(1)                                #Wait 1 second until next try
                else:
                    break
            _print('MAVLINK connection {0} acquired'.format(self._ident))
        def _mod(self):
            socket =  self._zmq_context.socket(zmq.PUB)
            socket.connect('inproc://in')                       #New socket which publish to the bridge
            self.try_connection()
            _print('MAVLINK_connection {0} loop start'.format(self._ident))
            while(self._run):
                try:
                    msg = self._mavh.recv_msg()                        #Blocking TBC
                except:
                    _print('MAVLINK connection {0} lost'.format(self._ident))
                    self.try_connection()
                else:
                    if msg is not None:
                        d_type = msg.get_type()
                        e_string  = ZMQ_MESSAGE_BINARY + '{0} {1:.3f} {2}'.format(self._ident, msg._timestamp,msg.get_msgbuf())
                        socket.send(e_string)
                        self._in_msg += 1
                        if (d_type != 'BAD DATA' and d_type != 'BAD_DATA'):      #BAD DATA message ignored
                            self._ok_msg += 1
                            data = {}
                            data[d_type] = {}
                            for i in msg.get_fieldnames():
                                data[d_type][i]=msg.__dict__[i]
                            try:
                                json_data = dumps(data)
                            except:
                                pass
                            else:
                                d_string = ZMQ_MESSAGE_JSON + '{0} {1:.3f} {2}'.format(self._ident, msg._timestamp , json_data)
                                self._out_msg += 1
                                socket.send(d_string)
            _print('MAVLINK_connection {0} loop stop'.format(self._ident))
            socket.close()
        def pymavlink_handle(self):
            return self._mavh
        def info(self):
            return {'ident' :  self._ident, 'argv': self._argv, 'kwargs': self._kwargs, 'msg_stats': {'in_msg': self._in_msg, 'ok_msg': self._ok_msg, 'out_msg': self._out_msg}}

class ZMQ_publisher(ModBase):
    def __init__(self, zmq_context, ident,  port):
        super(ZMQ_publisher, self).__init__()
        self._zmq_context = zmq_context
        self._ident = ident
        self._port = port
        self._out_msg = 0
    def _mod(self):
        socket_in = self._zmq_context.socket(zmq.SUB)
        socket_in.connect('inproc://out')                           #Connect to bridge output
        socket_in.setsockopt(zmq.SUBSCRIBE, '')                     #No filter
        socket_out = self._zmq_context.socket(zmq.PUB)              #Zmq publisher
        socket_out.bind("tcp://*:%s" % self._port)
        _print('ZMQ_publisher {0} loop start'.format(self._ident))
        while(self._run):
            string = socket_in.recv()
            socket_out.send(string)
            self._out_msg += 1
        socket_in.close()
        socket_out.close()
        _print('ZMQ_publisher {0} loop stop'.format(self._ident))
    def info(self):
            return {'ident' :  self._ident, 'port': self._port, 'msg_stats': {'out_msg': self._out_msg}}
           
class FILE_writer(ModBase):
    def __init__(self, zmq_context, ident, file):
        super(FILE_writer, self).__init__()
        self._zmq_context = zmq_context
        self._ident = ident
        self._file = file
        self._out_msg = 0
    def _mod(self):
        socket_in = self._zmq_context.socket(zmq.SUB)
        socket_in.connect('inproc://out')                               #Connect to bridge output
        socket_in.setsockopt(zmq.SUBSCRIBE, ZMQ_MESSAGE_JSON)           #Take only json data
        _print('FILE_writer {0} loop start'.format(self._ident))
        with open(self._file, 'w', 500) as f:
            while(self._run):
                string = socket_in.recv()
                print(string, file=f)
                self._out_msg += 1
        _print('FILE_writer {0} loop stop'.format(self._ident))
    def info(self):
        return {'ident' :  self._ident, 'file': self._file, 'msg_stats': {'out_msg': self._out_msg}}

class BIN_writer(ModBase):
    def __init__(self, zmq_context, ident, file):
        super(BIN_writer, self).__init__()
        self._zmq_context = zmq_context
        self._ident = ident
        self._file = file
        self._out_msg = 0
    def _mod(self):
        socket_in = self._zmq_context.socket(zmq.SUB)
        socket_in.connect('inproc://out')                               #Connect to bridge output
        socket_in.setsockopt(zmq.SUBSCRIBE, ZMQ_MESSAGE_BINARY)         #Take only binary data
        _print('FILE_writer {0} loop start'.format(self._ident))
        with open(self._file, 'wb', 500) as f:
            while(self._run):
                string = socket_in.recv()
                data = string.split(' ',2)[2]
                print(data, file=f,end='')
                self._out_msg += 1
        _print('BIN_writer {0} loop stop'.format(self._ident))
    def info(self):
        return {'ident' :  self._ident, 'file': self._file, 'msg_stats': {'out_msg': self._out_msg}}
        
class TCP_connection(ModBase):
    def __init__(self, zmq_context, ident, mav, address, port):
        super(TCP_connection, self).__init__()
        self._zmq_context = zmq_context
        self._zmq_socket = None
        self._ident = ident
        self._mav = mav
        self._address = (address, port)
        self._out_msg = 0
        self._in_msg = 0
        self._connection_h = None
        self._connection_activated = False
        self._connection_address = None
    def _mod(self):
        #ZMQ socket
        self._zmq_socket = self._zmq_context.socket(zmq.SUB)
        self._zmq_socket.connect('inproc://out')                           #Connect to bridge output
        self._zmq_socket.setsockopt(zmq.SUBSCRIBE, 'B' + self._mav.ident()) #Filter on encrypted message with mav ident
        #Regular socket
        r_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP
        r_socket.bind(self._address)
        while(self._run):
            if(self._connection_activated == False):
                if(self._connection_h != None):
                    self._connection_h.close()
                _print('TCP_connection {0} waiting '.format(self._ident))
                r_socket.listen(1)
                self._connection_h, self._connection_address = r_socket.accept()
                self._connection_activated = True
                _print('TCP_connection {0} : connected to {1}'.format(self._ident, self._connection_address))
                #Launch threaded functions
                self._mav_2_socket()
                self._socket_2_mav()
            sleep(1)
    @in_thread(True)
    def _mav_2_socket(self):
        _print('TCP_connection : mav to socket {0} loop start'.format(self._ident))
        while(self._connection_activated):
            string = self._zmq_socket.recv()
            out_data = string.split(' ',2)[2]
            try:
                self._connection_h.send(out_data)
                self._out_msg += 1
            except:
                self._connection_activated = False
    @in_thread(True)
    def _socket_2_mav(self):
        _print('TCP_connection : socket to mav {0} loop start'.format(self._ident))
        while(self._connection_activated):
            try:
                in_data = self._connection_h.recv(4096)
                self._in_msg += 1
            except:
                self._connection_activated = False
            else:
                py_h = self._mav.pymavlink_handle()
                if(py_h != None):
                    py_h.write(in_data)
    def info(self):
        return {'ident' :  self._ident, 'address': self._address, 'msg_stats': {'out_msg': self._out_msg,'in_msg': self._in_msg, 'connection activated': self._connection_activated}}
class Plug(object):
    '''Main class to manage the plug '''
    def __init__(self):
        print('Initialising Plug')
        self._zmq_context = zmq.Context()
        self._bridge = Bridge(self._zmq_context)
        self._bridge.run()
        self._input_list = []                               #Thread list for MAVLINK connection
        self._output_list = []                              #Thread list for other than MAVLINK connection
        self._verbose = False
    def MAVLINK_in(self, *argv, **kwargs):
        ident = '{0:02d}'.format(len(self._input_list))
        h = MAVLINK_connection(self._zmq_context, ident, *argv, **kwargs)
        h.run()
        self._input_list.append(h)
        return h
    def ZMQ_out(self, port):
        ident = '{0:02d}'.format(len(self._output_list))
        h = ZMQ_publisher(self._zmq_context, ident, port)
        h.run()
        self._output_list.append(h)
        return h
    def FILE_out(self, file):
        ident = '{0:02d}'.format(len(self._output_list))
        h = FILE_writer(self._zmq_context, ident, file)
        h.run()
        self._output_list.append(h)
        return h
    def BIN_out(self, file):
        ident = '{0:02d}'.format(len(self._output_list))
        h = BIN_writer(self._zmq_context, ident, file)
        h.run()
        self._output_list.append(h)
        return h
    def TCP_in_out(self, mavlink_connection, address, port):
        ident = '{0:02d}'.format(len(self._output_list))
        h = TCP_connection(self._zmq_context, ident, mavlink_connection, address, port)
        h.run()
        self._output_list.append(h)
        return h
    def verbose(self, switch):
        if(isinstance(switch, bool)):
            self._verbose = switch
    
    def server_forever(self):
        while(True):
            try:
                sleep(1)
                if(self._verbose):
                    for connection in self._input_list:
                        print('IN {0} {1}'.format(connection.info()['ident'], connection.info()['msg_stats']))
                    for connection in self._output_list:
                        print('OUT {0} {1}'.format(connection.info()['ident'], connection.info()['msg_stats']))
            except(KeyboardInterrupt, SystemExit):
                string = 'Exit called !!'
                print(string)
                print('Closing Plug')
                for conn in self._input_list + self._output_list:
                    conn.stop()
                self._bridge.stop()
                break
