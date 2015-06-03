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

#For now just available for V1.0 Pixhawk Message
import pymavlink.dialects.v10.pixhawk as mavlink
import struct

#TODO : Message Type definition
MSG_PLUG_ID_MAV_MSG=1

MSG_PLUG_TYPE = {
    MSG_PLUG_ID_MAV_MSG : MAVlinkPlugMessage
    }
#TODO : Special Destination, Source Definition
from mavlinkplug.Exception import MAVlinkPlugException 

class PlugMessage(object):
    
    def __init__(self, msg = None):
        if(msg == None):
            self._dest = None
            self._sour = None
            self._type = None
            self._mav_msg_data = None
        else:
            self._dest, self._sour, self._type  = struct.unpack('!BBB',msg)
            self._mav_msg_data = msg[3:]
        
    def set_header(self,dest, type, source):
        self._dest = dest
        self._sour = type
        self._type = source
        self._check_header()
    
    def _check_header(self):
        #Destination check
        if( not isinstance(self._dest, int) or self._dest < 0 or self._dest > 255):
            raise MAVlinkPlugException('Invalid header data : destination : {0}'.format(self._dest))
        #Source check
         if( not isinstance(self._sour, int) or self._sour < 0 or self._sour > 255):
            raise MAVlinkPlugException('Invalid header data : source : {0}'.format(self._sour))
        #Type check
        if( not self._type in MSG_PLUG_TYPE)): 
            raise MAVlinkPlugException('Invalid header data : type : {0}'.format(self._type))

    def set_data(self, data):
        self._mav_msg_data
    
    def get_plug_msg_buf(self):
        if(self._dest != None and self._sour != None and self._type != None):
            header = struct.pack('!BBB', self._dest, self._sour, self._type)
            msg = header + self._mav_msg_data
            return msg
        else:
            raise MAVlinkPlugException('Header not defined')

            
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
        self._mav_msg_data = m
        return m