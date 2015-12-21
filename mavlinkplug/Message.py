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

# Managing Mavlink dialect
import mavlinkplug
print('Importing ' + mavlinkplug._MAVLINKPLUG_DIALECT)
mavlink = __import__('pymavlink.dialects.v10.' + mavlinkplug._MAVLINKPLUG_DIALECT, globals(), locals(), [], -1)

#import pymavlink.dialects.v10.pixhawk as mavlink
from pymavlink.generator.mavcrc import x25crc
from mavlinkplug.Exception import Exception
from collections import namedtuple

#Data classes
class RawData(object):
    _type = 'RawData'
    def __init__(self, value = None):
        self._value = None #Useless
        self.value = value
    @property
    def packed(self):
        return self.value
    @property
    def value(self):
        if(self._value == None):
            raise Exception('Invalid value : value not define')
        else:
            return self._value
    @value.setter
    def value(self,value):
        #self._value contains raw value
        self._value = value
    @property
    def type(self):
        return self._type
    def build_from(self, value):        #For signature uniformity
        self.value = value
        return self
    @classmethod
    def build_full_message_from(cls, destination, source, timestamp, data):
        header = Header().build_from(destination, source, TYPE.RAW.value,timestamp)
        msg_data = cls().build_from(data)
        return Message().build_from(header, msg_data)

class MAVLinkData(RawData):
    _type = 'MAVLinkData'
    @property
    def packed(self):
        return self.value.get_msgbuf()
    @property
    def value(self):
        if(self._value == None):
            raise Exception('Invalid value : value not define')
        else:
            return self._value
    @value.setter
    def value(self,value):
        #self._value has to contain a MAVlink message class instance
        if(value != None):
            if(isinstance(value,mavlink.MAVLink)):     #Is this a mavlink message object ?
                self._value = value
            else:                                       #Try to decode into a mavlink message object
                self._value = self._decode(value)
        else:
            self._value = None
    @property
    def json(self):
        d_type = self.value.get_type()
        data = {}
        data[d_type] = {}
        if (d_type != 'BAD DATA' and d_type != 'BAD_DATA'):      #BAD DATA message ignored
            for i in self.data.get_fieldnames():
                data[d_type][i]=self.value.__dict__[i]
                json_data = dumps(data)
        return data
    #Decode buffer into MAVlink message class instance
    def _decode(self, msgbuf):
        '''
        Decode a buffer as a MAVLink message
        '''
        try:
            magic, mlen, seq, srcSystem, srcComponent, msgId = struct.unpack('cBBBBB', msgbuf[:6])
        except struct.error as emsg:
            raise Exception('Unable to unpack MAVLink header: %s' % emsg)
        if ord(magic) != 254:
            raise Exception("invalid MAVLink prefix '%s'" % magic)
        if mlen != len(msgbuf)-8:
            raise Exception('invalid MAVLink message length. Got %u expected %u, msgId=%u' % (len(msgbuf) - 8, mlen, msgId))
        if not msgId in mavlink.mavlink_map:
            raise Exception('unknown MAVLink message ID %u' % msgId)

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
            raise Exception('Unable to unpack MAVLink CRC: %s' % emsg)
        crcbuf = msgbuf[1:-2]
        crcbuf = crcbuf + struct.pack('B',crc_extra)
        crc2 = x25crc(crcbuf)
        if crc != crc2.crc:
            raise Exception('invalid MAVLink CRC in msgID %u 0x%04x should be 0x%04x' % (msgId, crc, crc2.crc))

        try:
            t = struct.unpack(fmt, msgbuf[6:-2])
        except struct.error as emsg:
            raise Exception('Unable to unpack MAVLink payload type=%s fmt=%s payloadLength=%u: %s' % (
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
            raise Exception('Unable to instantiate MAVLink message of type %s : %s' % (type, emsg))
        m._msgbuf = msgbuf
        m._payload = msgbuf[6:-2]
        m._crc = crc
        m._header = mavlink.MAVLink_header(msgId, mlen, seq, srcSystem, srcComponent)
        self._value = m
        return m

class MavCommandData(RawData):
    _type = 'MavCommandData'
    pass

class KillData(RawData):
    _type = 'KillData'
    pass

class LogData(RawData):
    _type = 'LogData'
    pass

class TypeContainer(object):
    #Class attributes
    _type_description = [
        ['MAV_MSG', 1, MAVLinkData],
        ['MAV_COMMAND', 2, MavCommandData],
        ['KILL', 4, KillData],
        ['RAW',8, RawData],
        ['LOG_DATA',16, LogData]
    ]
    TypeItem = namedtuple('TypeItem', ['value','p_value','m_class'])

    def __init__(self):
        self._PACK_FORMAT = '!B'
        self._names = []
        self._values = []
        self._p_values = []
        self._m_classes = []
        for value in self._type_description :
            self._names.append(value[0])
            self._values.append(value[1])
            self._p_values.append(struct.pack(self._PACK_FORMAT,value[1]))
            self._m_classes.append(value[2])
    @property
    def values(self):
        return self._values
    @property
    def p_values(self):
        return self._p_values
    def __getattr__(self, name):
        if(name in self._names):
            return self.TypeItem(self._values[self._names.index(name)], self._p_values[self._names.index(name)], self._m_classes[self._names.index(name)])
        else:
            raise Exception('Message Type {0} not defined'.format(name))
    def get_class_from_value(self, p_value):
        return self._get_X_from_Y(p_value, self._m_classes, self._values)
    def get_class_from_p_value(self, p_value):
        return self._get_X_from_Y(p_value, self._m_classes, self._p_values)
    def get_value_from_class(self, m_class):
        return self._get_X_from_Y(m_class,self._values, self._m_classes)
    def _get_X_from_Y(self,value, X_table, Y_table):
        if(value in Y_table):
            return  X_table[Y_table.index(value)]
        else:
            raise Exception('Message Type search item not defined'.format(value))

class DestinationContainer(object):
    _destination_description = [
        ['ALL', 255],
    ]
    DestinationItem = namedtuple('DestinationItem',['value','p_value'])

    def __init__(self):
        self._PACK_FORMAT = '!B'
        self._names = []
        self._values = []
        self._p_values = []
        for value in self._destination_description :
            self._names.append(value[0])
            self._values.append(value[1])
            self._p_values.append(struct.pack(self._PACK_FORMAT,value[1]))
    def __getattr__(self, name):
        if(name in self._names):
            return self.DestinationItem(self._values[self._names.index(name)], self._p_values[self._names.index(name)])
        else:
            raise Exception('Message Destination {0} not defined'.format(name))

TYPE = TypeContainer()
DESTINATION = DestinationContainer()

# Header
class Header(object):
    def __init__(self):
        self._pack = '!BBBQ'
        self._destination = None
        self._source = None
        self._type = None
        self._timestamp = None

    #Destination property
    @property
    def destination(self):
        if(self._destination == None):
            raise Exception('Invalid header data : destination not define')
        else:
            return self._destination
    @destination.setter
    def destination(self, destination):
        if( not isinstance(destination, int) or destination < 0 or destination > 255):
            raise Exception('Invalid header destination set value: {0}'.format(destination))
        else:
            self._destination = destination
    #Source property
    @property
    def source(self):
        if(self._source == None):
            raise Exception('Invalid header data : source not define')
        else:
            return self._source
    @source.setter
    def source(self, source):
        if( not isinstance(source, int) or source < 0 or source > 255):
            raise Exception('Invalid header source set value: {0}'.format(source))
        else:
            self._source = source
    #Type property
    @property
    def type(self):
        if(self._type == None):
            raise Exception('Invalid header data : type not define')
        else:
            return self._type
    @type.setter
    def type(self, type):
        if( not type in TYPE.values):
            raise Exception('Invalid header type set value: {0}'.format(type))
        else:
            self._type = type
    #Timestamp property
    @property
    def timestamp(self):
        if(self._timestamp == None):
            raise Exception('Invalid header data : timestamp not define')
        else:
            return self._timestamp
    @timestamp.setter
    def timestamp(self, timestamp):
        if(timestamp < 0):
            raise Exception('Invalid header timestamp set value: {0}'.format(timestamp))
        else:
            self._timestamp = timestamp
    #Packed property
    @property
    def packed(self):
        return struct.pack(self._pack, self.destination, self.source, self.type, self.timestamp)
    #Build header from parameter
    def build_from(self, destination, source, type, timestamp):
        self.destination = destination
        self.source = source
        self.type = type
        self.timestamp = timestamp
        return self
    #Unpack header from message
    def unpack_from(self, message):
        p_size = struct.Struct(self._pack).size
        self.destination, self.source, self.type, self.timestamp = struct.unpack(self._pack ,message[:p_size])
        return TYPE.get_class_from_value(self.type)(message[p_size:])

#Message
class Message(object):
    def __init__(self):
        self._header = None
        self._data = None
    @property
    def data(self):
        if(self._data == None):
            raise Exception('Invalid Data : data not define')
        else:
            return self._data
    @data.setter
    def data(self,value):
        if(self._header == None):
            raise Exception('Invalid Header : header has to be define ahead of data')
        else:
            #self._data need to contain data class instance
            #header type  must match data type
            self.header.type = TYPE.get_value_from_class(type(value))
            self._data = value
    @property
    def header(self):
        if(self._header == None):
            raise Exception('Invalid Header : header not define')
        else:
            return self._header
    @header.setter
    def header(self,value):
        #self._header need to contain header class instance
        if(type(value) == type(Header())):
            self._header = value
        else:
            raise Exception('Invalid Header : object is not an instance of Header class')
    @property
    def packed(self):
        return self.header.packed + self.data.packed
    def unpack_from(self, byte_message):
        self.header = Header()
        self.data = self.header.unpack_from(byte_message) #Check included !!!!
        return self
    def build_from(self, header_instance, data_instance):
        self.header = header_instance
        self.data = data_instance
        return self
def integer_pack(_integer):
    return struct.pack('!B', _integer)