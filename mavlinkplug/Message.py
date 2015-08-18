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
#For now just available for V1.0 Ardupilot  Message
import pymavlink.dialects.v10.ardupilotmega as mavlink
#import pymavlink.dialects.v10.pixhawk as mavlink
from pymavlink.generator.mavcrc import x25crc
from mavlinkplug.Exception import MAVlinkPlugException 


#Data classes
class RawData(object):
    _type = 'RawData'
    def __init__(self, value = None):
        self._data = None #Useless
        self.data = value
    @property
    def packed(self):
        return self.data
    @property
    def data(self):
        if(self._data == None):
            raise MAVlinkPlugException('Invalid Data : data not define')
        else:
            return self._data
    @data.setter
    def data(self,value):
        #self._data contains raw data
        self._data = value
    @property
    def type(self):
        return self._type

class MAVLinkData(RawData):
    _type = 'MAVLinkData'
    @property
    def packed(self):
        return self.data.get_msgbuf()
    @property
    def data(self):
        if(self._data == None):
            raise MAVlinkPlugException('Invalid Data : data not define')
        else:
            return self._data
    @data.setter
    def data(self,value):
        if(value != None):
            #self._data contains a MAVlink message class instance
            self._data = self._decode(value)
    @property
    def json(self):
        d_type = self.data.get_type()
        data = {}
        data[d_type] = {}
        if (d_type != 'BAD DATA' and d_type != 'BAD_DATA'):      #BAD DATA message ignored
            for i in self.data.get_fieldnames():
                data[d_type][i]=self.data.__dict__[i]
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

 {
        'MAV_MSG': [ 1, MAVLinkData],
        'MAV_COMMAND': [2, RawData],
        'KILL': [4, RawData],
        'RAW': [8, RawData]
    }

class TypeContainer(object):
    def __init__(self, structure, data):
        class _Item(object):
            pass
        for key, value in data.iteritems() :
            temp = _Item()
            for prop_name in structure:
                temp.__dict__[prop_name] = value[structure.index(prop_name)]
            self.__dict__[key] = temp

    def __getattr__(self, name):
        if(name in self._name):
            return cls._clist.index(name)
        elif(name.endswith('_P')):
            return pack(_PACK_FORMAT, cls.__getattr__(name[:-2]))
        else:
            raise MAVlinkPlugException('MSG_PLUG_TYPE {0} not existing'.format(name))
    def __contains__(cls, item):
        return True if(item in cls._clist) else False


class Destination(object):
    # Message Type definition
    _type = [
        ['ALL', 255, None]
    ]


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
            raise MAVlinkPlugException('Invalid header data : destination not define')
        else:
            return self._destination
    @destination.setter
    def destination(self, destination):
        if( not isinstance(destination, int) or destination < 0 or destination > 255):
            raise MAVlinkPlugException('Invalid header destination set value: {0}'.format(destination))
        else:
            self._destination = destination
    #Source property
    @property
    def source(self):
        if(self._source == None):
            raise MAVlinkPlugException('Invalid header data : source not define')
        else:
            return self._source
    @source.setter
    def source(self, source):
        if( not isinstance(source, int) or source < 0 or source > 255):
            raise MAVlinkPlugException('Invalid header source set value: {0}'.format(source))
        else:
            self._source = source
    #Type property
    @property
    def type(self):
        if(self._type == None):
            raise MAVlinkPlugException('Invalid header data : type not define')
        else:
            return self._type
    @type.setter
    def type(self, type):
        if( not type in MSG_PLUG_TYPE):
            raise MAVlinkPlugException('Invalid header type set value: {0}'.format(type))
        else:
            self._type = type
    #Timestamp property
    @property
    def timestamp(self):
        if(self._timestamp == None):
            raise MAVlinkPlugException('Invalid header data : timestamp not define')
        else:
            return self._timestamp
    @timestamp.setter
    def timestamp(self, timestamp):
        if( not isinstance(timestamp, long) or timestamp < 0):
            raise MAVlinkPlugException('Invalid header timestamp set value: {0}'.format(timestamp))
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
    #Unpack header from message
    def unpack_from(self, message):
        p_size = struct.Struct(self._pack).size
        self.destination, self.source, self.type, self.timestamp = struct.unpack(self._pack ,message[:p_size])
        #TODO: has to return an instance of data class
        return message[p_size:]


#Message
class Message(object):
    def __init__(self):
        self._header = None
        self._data = None
    @property
    def data(self):
        if(self._data == None):
            raise MAVlinkPlugException('Invalid Data : data not define')
        else:
            return self._data
    @data.setter
    def data(self,value):
        if(self._header == None):
            raise MAVlinkPlugException('Invalid Header : header has to be define ahead of data')
        else:
            #self._data need to contain data class instance
            #TODO : Add data check against header type
            self._data = value
    @property
    def header(self):
        if(self._header == None):
            raise MAVlinkPlugException('Invalid Header : header not define')
        else:
            return self._header
    @header.setter
    def header(self,value):
        #self._header need to contain header class instance
        self._header = value
    @property
    def packed(self):
        return self.header.packed + self.data.packed
    def unpack_from(self, byte_message):
        self.header = Header()
        self.data = self.header.unpack_from(byte_message) #Check included !!!!
    def build_from(self, header_instance, data_instance):
        self.header = header_instance
        self.data = data_instance

