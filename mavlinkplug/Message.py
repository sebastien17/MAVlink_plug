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

import struct
from json import dumps
#For now just available for V1.0 Pixhawk Message
import pymavlink.dialects.v10.ardupilotmega as mavlink
#import pymavlink.dialects.v10.pixhawk as mavlink
from pymavlink.generator.mavcrc import x25crc
from mavlinkplug.Exception import MAVlinkPlugException 


#TODO : Message Type definition
MSG_PLUG_TYPE_MAV_MSG = 1
MSG_PLUG_TYPE_MAV_COMMAND = 13
MSG_PLUG_TYPE_KILL = 17

MSG_PLUG_TYPE = {
    MSG_PLUG_TYPE_MAV_MSG : 'MAVlinkPlugMessage',
    MSG_PLUG_TYPE_MAV_COMMAND : 'MAVlinkPlugMavCommand',
    MSG_PLUG_TYPE_KILL : 'MAVlinkPlugKill'
    }
    
#TODO : Special Destination, Source Definition
MSG_PLUG_DEST_TYPE_ALL = 255

def DEF_PACK(_integer):
    return struct.pack('!B', _integer)

class PlugHeader(object):
    def __init__(self, destination = None, source = None, type = None, timestamp = 0.0):
        self.__pack = '!BBBQ'
        self.__destination = destination
        self.__source = source
        self.__type = type
        self.__timestamp = timestamp
    
    #Destination property
    @property
    def destination(self):
        if(self.__destination == None):
            raise MAVlinkPlugException('Invalid header data : destination not define')
        else:
            return self.__destination
    @destination.setter
    def destination(self, destination):
        if( not isinstance(destination, int) or destination < 0 or destination > 255):
            raise MAVlinkPlugException('Invalid header destination set value: {0}'.format(destination))
        else:
            self.__destination = destination
    
    #Source property
    @property
    def source(self):
        if(self.__source == None):
            raise MAVlinkPlugException('Invalid header data : source not define')
        else:
            return self.__source
    @source.setter
    def source(self, source):
        if( not isinstance(source, int) or source < 0 or source > 255):
            raise MAVlinkPlugException('Invalid header source set value: {0}'.format(source))
        else:
            self.__source = source
    
    #Type property
    @property
    def type(self):
        if(self.__type == None):
            raise MAVlinkPlugException('Invalid header data : type not define')
        else:
            return self.__type
    @type.setter
    def type(self, type):
        if( not type in MSG_PLUG_TYPE):
            raise MAVlinkPlugException('Invalid header type set value: {0}'.format(type))
        else:
            self.__type = type
            
    #Timestamp property
    @property
    def timestamp(self):
        if(self.__timestamp == None):
            raise MAVlinkPlugException('Invalid header data : timestamp not define')
        else:
            return self.__timestamp
    @timestamp.setter
    def timestamp(self, timestamp):
        if(  not isinstance(timestamp, int) or timestamp < 0):
            raise MAVlinkPlugException('Invalid header timestamp set value: {0}'.format(timestamp))
        else:
            self.__timestamp = timestamp
    def extract(self, msg):
        self.destination, self.source, self.type, self.timestamp = struct.unpack(self.__pack ,msg[:self.size()])
        return msg[self.size():]
    def pack(self):
        return struct.pack(self.__pack, self.__destination, self.__source, self.__type, self.__timestamp)
    def size(self):
        return struct.Struct(self.__pack).size
        
        
class PlugMessage(object):
    def __init__(self, msg = None):
        self.__header = PlugHeader()
        if(msg == None):
            self.__data = None
        else:
             self.data = self.__header.extract(msg)
    #header property
    @property
    def header(self):
        return self.__header
    @header.setter
    def header(self, value):
        self.__header = value
    #data property
    @property
    def data(self):
        if(self.__data == None):
            raise MAVlinkPlugException('No data available')
        else:
            return self.__data
    @data.setter
    def data(self, value):
        #TODO : add check on this setter
        self.__data = value
    def pack(self):
        return self.__header.pack() + self.__data
        
class MAVlinkPlugMessage(PlugMessage):
    @property
    def data(self):
        if(self.__data == None):
            raise MAVlinkPlugException('No data available')
        else:
            return self.__data
    @data.setter
    def data(self, value):
        #TODO : add check on this setter
        #Set a MAVLink msg class into __data
        self.__data = self.decode(value)
    def pack(self):
        return self.__header.pack() + self.__data.get_msgbuf()
    def json_mav_msg(self):
        if(self.__data is None):
            raise MAVlinkPlugException('No data available')
        else:
            d_type = self.__data.get_type()
            data = {}
            data[d_type] = {}
            if (d_type != 'BAD DATA' and d_type != 'BAD_DATA'):      #BAD DATA message ignored
                for i in self.__data.get_fieldnames():
                    data[d_type][i]=self.__data.__dict__[i]
                    json_data = dumps(data)
            return data
    def decode(self, msgbuf):
        '''
        Decode a buffer as a MAVLink message
        '''
        try:
            magic, mlen, seq, srcSystem, srcComponent, msgId = struct.unpack('cBBBBB', msgbuf[:6])
        except struct.error as emsg:
            raise MAVlinkPlugException('Unable to unpack MAVLink header: %s' % emsg)
        if ord(magic) != 254:
            raise MAVlinkPlugException("invalid MAVLink prefix '%s'" % magic)
        if mlen != len(msgbuf)-8:
            raise MAVlinkPlugException('invalid MAVLink message length. Got %u expected %u, msgId=%u' % (len(msgbuf)-8, mlen, msgId))
        if not msgId in mavlink.mavlink_map:
            raise MAVlinkPlugException('unknown MAVLink message ID %u' % msgId)

        # decode the payload
        type = mavlink.mavlink_map[msgId]
        fmt = type.format
        order_map = type.orders
        len_map = type.lengths
        crc_extra = type.crc_extra

        # decode the checksum
        try:
            crc, = struct.unpack('<H', msgbuf[-2:])
        except struct.error as emsg:
            raise MAVlinkPlugException('Unable to unpack MAVLink CRC: %s' % emsg)
        crcbuf = msgbuf[1:-2]
        crcbuf = crcbuf + struct.pack('B',crc_extra)
        crc2 = x25crc(crcbuf)
        if crc != crc2.crc:
            raise MAVlinkPlugException('invalid MAVLink CRC in msgID %u 0x%04x should be 0x%04x' % (msgId, crc, crc2.crc))

        try:
            t = struct.unpack(fmt, msgbuf[6:-2])
        except struct.error as emsg:
            raise MAVlinkPlugException('Unable to unpack MAVLink payload type=%s fmt=%s payloadLength=%u: %s' % (
                type, fmt, len(msgbuf[6:-2]), emsg))

        tlist = list(t)
        # handle sorted fields
        if True:
            t = tlist[:]
            if sum(len_map) == len(len_map):
                # message has no arrays in it
                for i in range(0, len(tlist)):
                    tlist[i] = t[order_map[i]]
            else:
                # message has some arrays
                tlist = []
                for i in range(0, len(order_map)):
                    order = order_map[i]
                    L = len_map[order]
                    tip = sum(len_map[:order])
                    field = t[tip]
                    if L == 1 or isinstance(field, str):
                        tlist.append(field)
                    else:
                        tlist.append(t[tip:(tip + L)])

        # terminate any strings
        for i in range(0, len(tlist)):
            if isinstance(tlist[i], str):
                tlist[i] = str(mavlink.MAVString(tlist[i]))
        t = tuple(tlist)
        # construct the message object
        try:
            m = type(*t)
        except Exception as emsg:
            raise MAVlinkPlugException('Unable to instantiate MAVLink message of type %s : %s' % (type, emsg))
        m._msgbuf = msgbuf
        m._payload = msgbuf[6:-2]
        m._crc = crc
        m._header = mavlink.MAVLink_header(msgId, mlen, seq, srcSystem, srcComponent)
        self._data = m
        return m