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
import tornado.ioloop
import tornado.tcpserver

#Internal Module
from mavlinkplug.Base import ZmqBase
import mavlinkplug.Message
import mavlinkplug.Exception

class Connection(object):
    clients = set()

    def __init__(self, stream, address, zmqprocess):
        Connection.clients.add(self)
        self._stream = stream
        self._address = address
        self._zmqprocess = zmqprocess
        self._stream.set_close_callback(self.on_close)
        self.read_message()

    def read_message(self):
        self._stream.read_until_close(streaming_callback=self._zmqprocess)

    @classmethod
    def broadcast_messages(cls, b_data):
        for conn in cls.clients:
            conn.send_message(b_data)

    def send_message(self, data):
        self._stream.write(data)

    def on_close(self):
        Connection.clients.remove(self)


class TcpGsServer(tornado.tcpserver.TCPServer):
    def __init__(self, zmqprocess, **kwargs):
        super(TcpGsServer, self).__init__(**kwargs)
        self._zmqprocess = zmqprocess

    def handle_stream(self, stream, address):
        print ("New connection {0}:".format(address))
        Connection(stream, address, self._zmqprocess)

    def broadcast(self, data):
        Connection.broadcast_messages(data)

class TcpConnection(ZmqBase):
    def __init__(self, module_info, tcp_tuple, mav_id, name = None):
        super(TcpConnection, self).__init__()
        self._addr_to_plug, self._addr_from_plug, self._ident =  module_info
        self._tcp_tuple = tcp_tuple
        self._mav_connection_ident = mav_id
        self._default_subscribe.append(mavlinkplug.Message.integer_pack(self._ident))
        self._socket = None
        self._tcpserver = None
        if not name:
            self._name = 'TcpConnection_' + str(self._ident)
        else:
            self._name = name

    def setup(self):
        super(TcpConnection, self).setup()
        #Create TCP Server
        self._tcpserver = TcpGsServer(self._process_udp_stream)
        self._tcpserver.listen(self._tcp_tuple[1], self._tcp_tuple[0])
        #Create ZMQ socket
        self._stream2Plug = self.stream(zmq.PUB, self._addr_to_plug, bind=False)
        self._streamFromPlug = self.stream(zmq.SUB, self._addr_from_plug, bind=False, callback=self._process_zmq_stream)

    def _process_udp_stream(self, data):
        _mavlink_plug_message = mavlinkplug.Message.Message()
        _mavlink_plug_message.header = mavlinkplug.Message.Header().build_from( self._mav_connection_ident,
                                                                                self._ident,
                                                                                mavlinkplug.Message.TYPE.RAW.value,
                                                                                1L)
        _mavlink_plug_message.header.timestamp = long(time.time())
        try:
            _mavlink_plug_message.data =  mavlinkplug.Message.MAVLinkData(data)
        except mavlinkplug.Exception.Exception:
            _mavlink_plug_message.data =  mavlinkplug.Message.RawData(data)
        self._stream2Plug.send(_mavlink_plug_message.packed)

    def _process_zmq_stream(self, in_data):
        in_data = in_data[0]
        mavlinkplug_message = mavlinkplug.Message.Message().unpack_from(in_data)
        print(type(mavlinkplug_message.data.packed))
        if mavlinkplug_message.header.type == mavlinkplug.Message.TYPE.MAV_MSG.value and mavlinkplug_message.header.source == self._mav_connection_ident:
            self._tcpserver.broadcast(mavlinkplug_message.data.packed)

    def _logging(self, msg, type = 'INFO'):
        log_message = mavlinkplug.Message.LogData.build_full_message_from(  mavlinkplug.Message.DESTINATION.ALL.value,
                                                                                self._ident,
                                                                                long(time.time()),
                                                                                type+': '+msg)
        self._stream2Plug.send(log_message.packed)