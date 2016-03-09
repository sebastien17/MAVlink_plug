#!/usr/bin/env python
# -*- coding: utf-8 -*-

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# This file is part of MAVlinkplug.

# MAVlinkplug is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# MAVlinkplug is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with MAVlinkplug.  If not, see <http://www.gnu.org/licenses/>.
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#Core Module
from __future__ import print_function
from time import sleep, time

#External Module
import zmq
import socket
import multiprocessing

#Internal Module
import  mavlinkplug.Message
from mavlinkplug.Base import in_thread
from mavlinkplug.Exception import Exception

class TcpConnection(multiprocessing.Process):
    def __init__(self, module_info, tcp_tuple, mav_connection_ident, name = None):
        super(TcpConnection, self).__init__()
        self._addr_to_plug, self._addr_from_plug, self._ident =  module_info
        self._stream2Plug = None
        self._streamFromPlug = None
        self._tcp_tuple = tcp_tuple
        self._mav_connection_ident = mav_connection_ident
        self._run = True
        self._subscribe = ['']
        if(name == None ):
            self._name = 'TCPConnection_' + str(self._ident)
        else:
            self._name = name
    def run(self):
        self._connection_activated = False
        self._connection_h = None
        self._zmq_context = zmq.Context()
        #Create TCP socket
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP
        self._socket.bind(self._tcp_tuple)
        while(self._run):
            if(self._connection_activated == False):
                if(self._connection_h != None):
                    self._connection_h.close()
                self._logging('Waiting')
                self._socket.listen(1)
                self._connection_h, self._connection_address = self._socket.accept()
                self._connection_activated = True
                self._logging('Connected to {0}'.format(self._connection_address))
                #Launch threaded functions
                self._ZMQ_2_UDP()
                self._UDP_2_ZMQ()
            sleep(1)
    @in_thread()
    def _ZMQ_2_UDP(self):
        #Create ZMQ socket
        _zmq_from_plug = self._zmq_context.socket(zmq.SUB)
        for opt in self._subscribe:
                _zmq_from_plug.setsockopt(zmq.SUBSCRIBE, opt)
        _zmq_from_plug.connect(self._addr_from_plug)
        while(self._connection_activated):
            mavlinkplug_message = mavlinkplug.Message.Message().unpack_from(_zmq_from_plug.recv())
            if(mavlinkplug_message.header.type == mavlinkplug.Message.TYPE.MAV_MSG.value and mavlinkplug_message.header.source == self._mav_connection_ident):
                try:
                    self._connection_h.send(mavlinkplug_message.data.packed)
                except:
                    self._connection_activated = False
    @in_thread()
    def _UDP_2_ZMQ(self):
        #Create ZMQ socket
        _zmq_to_plug = self._zmq_context.socket(zmq.PUB)
        _zmq_to_plug.connect(self._addr_to_plug)
        _header = mavlinkplug.Message.Header().build_from(self._mav_connection_ident,
                                                          self._ident,
                                                          mavlinkplug.Message.TYPE.RAW.value,
                                                          1L)
        _mavlink_plug_message = mavlinkplug.Message.Message()
        _mavlink_plug_message.header = _header
        while(self._connection_activated):
            try:
                _in_data = self._connection_h.recv(1024)
            except:
                self._connection_activated = False
            else:
                _mavlink_plug_message.header.timestamp = long(time()*1000)
                try:
                    _mavlink_plug_message.data =  mavlinkplug.Message.MAVLinkData(_in_data)
                except(Exception) :
                    _mavlink_plug_message.data =  mavlinkplug.Message.RawData(_in_data)
                _zmq_to_plug.send(_mavlink_plug_message.packed)
    def stop(self):
        self._run = False
        self._logging('Closing')
        self.terminate()
    def _logging(self, msg, type = 'INFO'):
         pass
         #logging_message = mavlinkplug.Message.LogData.build_full_message_from( 0, self._ident, long(time()), type+': '+ msg )
         #self._stream2Plug.send(logging_message.packed)
