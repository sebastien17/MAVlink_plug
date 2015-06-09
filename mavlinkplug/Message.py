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
#For now just available for V1.0 Pixhawk Message
import pymavlink.dialects.v10.pixhawk as mavlink
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
        self._pack = '!BBBQ'
        self.destination = destination
        self.source = source
        self.type = type
        self.timestamp = timestamp
    def set_header(self, destination, source, type, timestamp):
        self.destination = destination
        self.source = source
        self.type = type
        self.timestamp = timestamp
        self._check_header()
    def _check_header(self):
        #destinationination check
        if( not isinstance(self.destination, int) or self.destination < 0 or self.destination > 255):
            raise MAVlinkPlugException('Invalid header data : destinationination : {0}'.format(self.destination))
        #Source check
        if( not isinstance(self.source, int) or self.source < 0 or self.source > 255):
            raise MAVlinkPlugException('Invalid header data : source : {0}'.format(self.source))
        #Type check
        if( not self.type in MSG_PLUG_TYPE): 
            raise MAVlinkPlugException('Invalid header data : type : {0}'.format(self.type))
        #Timestamp check
        if( not isinstance(self.timestamp, float) or self.timestamp < 0):
            raise MAVlinkPlugException('Invalid header data : timestamp : {0}'.format(self.timestamp))
    def extract(self, msg):
        _dest, _sour, _type, _timestamp, _msg_data = struct.unpack(self._pack + 's',msg)
        self.set_header(_dest, _sour, _type, _timestamp)
        return _msg_data
    def pack(self):
        self._check_header()
        return struct.pack(self._pack, self.destination, self.source, self.type, self.timestamp)
        
class PlugMessage(object):
    def __init__(self, msg = None):
        if(msg == None):
            self._header = PlugHeader()
            self._data = None
        else:
             self._data = self._header.extract(msg)
    def set_header(self,**kwargs):
        self._header.set_header(**kwargs)
    def set_data(self, data):
        self._data = data
    def get_type(self):
        return self._header.type
    def get_source(self):
        return self._header.sour
    def get_destination(self):
        return self._header.dest
    def get_timestamp(self):
        return self._header.timestamp
    def get_data(self):
        return self._data
    def pack(self):
        header = self._header.pack()
        msg = header + self._data
        return msg
    def get_unpacked_data(self):
        if(self._data == None):
            raise MAVlinkPlugException('No PlugMessage data')
        
        
        
class MAVlinkPlugMessage(PlugMessage):
    def decode(self, msgbuf):
        '''decode a buffer as a MAVLink message'''
        # decode the header
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