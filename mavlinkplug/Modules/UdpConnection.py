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
import time

#External Module
import zmq
import socket
import tornado.ioloop
from tornado import gen

#Internal Module
from mavlinkplug.Base import ZmqBase
import mavlinkplug.Message
import mavlinkplug.Exception

class UDPStream(object):
    def __init__(self, socket):
        self.socket = socket
        self._state = None
        self._read_callback = None
        self.ioloop = tornado.ioloop.IOLoop.instance().current()
        self._read_timeout = None

    def _add_io_state(self, state):
        if self._state is None:
            self._state = tornado.ioloop.IOLoop.ERROR | state
            self.ioloop.add_handler(
                self.socket.fileno(), self._handle_events, self._state)
        elif not self._state & state:
            self._state = self._state | state
            self.ioloop.update_handler(self.socket.fileno(), self._state)

    def sendto(self,msg,address):
        return self.socket.sendto(msg, address)

    def recv(self,sz):
        return self.socket.recv(sz)

    def close(self):
        self.ioloop.remove_handler(self.socket.fileno())
        self.socket.close()
        self.socket = None

    def read_chunk(self, callback=None, timeout=4):
        self._read_callback = callback
        self._read_timeout = self.ioloop.add_timeout( time.time() + timeout,
            self.check_read_callback )
        self._add_io_state(self.ioloop.READ)

    def check_read_callback(self):
        if self._read_callback:
            # XXX close socket?
            self._read_callback(None, error='timeout')

    def _handle_read(self):
        if self._read_timeout:
            self.ioloop.remove_timeout(self._read_timeout)
        if self._read_callback:
            try:
                data = self.socket.recv(4096)
            except:
                # conn refused??
                data = None
            self._read_callback(data)
            self._read_callback = None

    def _handle_events(self, fd, events):
        if events & self.ioloop.READ:
            self._handle_read()
        if events & self.ioloop.ERROR:
            print('%s event error' % self)


class UdpConnection(ZmqBase):
    def __init__(self, module_info, tcp_tuple, mav_id, name = None):
        super(UdpConnection, self).__init__()
        self._addr_to_plug, self._addr_from_plug, self._ident =  module_info
        self._tcp_tuple = tcp_tuple
        self._mav_connection_ident = mav_id
        self._default_subscribe.append( mavlinkplug.Message.integer_pack(self._ident))
        self._socket = None
        self._udpstream = None
        if not name:
            self._name = 'UdpConnection_' + str(self._ident)
        else:
            self._name = name

    def setup(self):
        super(UdpConnection, self).setup()
        #Create UDP socket
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)     #UDP
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.setblocking(0)                                         #Non-blocking
        self._socket.bind(self._tcp_tuple)                                  # Bind to address
        self._udpstream = UDPStream(self._socket)
        self._loop.add_callback(self._process_udp_stream)
        #Create ZMQ socket
        self._stream2Plug = self.stream(zmq.PUB, self._addr_to_plug, bind=False)
        self._streamFromPlug = self.stream(zmq.SUB, self._addr_from_plug, bind=False, callback=self._process_zmq_stream)

    def _process_udp_stream(self):
        while True:
            _in_data = yield gen.Task(self._udpstream.read_chunk)
            _mavlink_plug_message = mavlinkplug.Message.Message()
            _mavlink_plug_message.header = mavlinkplug.Message.Header().build_from( self._mav_connection_ident,
                                                                                    self._ident,
                                                                                    mavlinkplug.Message.TYPE.RAW.value,
                                                                                    1L)
            _mavlink_plug_message.header.timestamp = long(time.time())
            try:
                _mavlink_plug_message.data =  mavlinkplug.Message.MAVLinkData()
            except mavlinkplug.Exception.Exception:
                _mavlink_plug_message.data =  mavlinkplug.Message.RawData(_in_data)
            self._stream2Plug.send(_mavlink_plug_message.packed)

    def _process_zmq_stream(self, in_data):
        in_data = in_data[0]
        mavlinkplug_message = mavlinkplug.Message.Message().unpack_from(in_data)
        if mavlinkplug_message.header.type == mavlinkplug.Message.TYPE.MAV_MSG.value and mavlinkplug_message.header.source == self._mav_connection_ident:
            self._udpstream.sendto(mavlinkplug_message.data.packed, self._tcp_tuple)

    def _logging(self, msg, type = 'INFO'):
        log_message = mavlinkplug.Message.LogData.build_full_message_from(  mavlinkplug.Message.DESTINATION.ALL.value,
                                                                                self._ident,
                                                                                long(time.time()),
                                                                                type+': '+msg)
        self._stream2Plug.send(log_message.packed)