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


#TODO : Message Type definition
MSG_PLUG_TYPE_MAV_MSG = 1
MSG_PLUG_TYPE_MAV_COMMAND = 13
MSG_PLUG_TYPE_KILL = 17
MSG_PLUG_TYPE_RAW = 31

MSG_PLUG_TYPE = {
    MSG_PLUG_TYPE_MAV_MSG : 'MAVlinkPlugMessage',
    MSG_PLUG_TYPE_MAV_COMMAND : 'MAVlinkPlugMavCommand',
    MSG_PLUG_TYPE_KILL : 'MAVlinkPlugKill',
    MSG_PLUG_TYPE_RAW : 'MAVlinkPlugRaw'
    }
    
#TODO : Special Destination, Source Definition
MSG_PLUG_DEST_TYPE_ALL = 255

def DEF_PACK(_integer):
    return struct.pack('!B', _integer)


    

    def extract(self, msg):
        self.destination, self.source, self.type, self.timestamp = struct.unpack(self._pack ,msg[:self.size()])
        return msg[self.size():]
    def size(self):
        return struct.Struct(self._pack).size

class PlugMessage(object):
    def __init__(self, msg = None):
        self._header = PlugHeader()
        if(msg == None):
            self._data = None
        else:
             self.data = self._header.extract(msg)
    #header property
    @property
    def header(self):
        return self._header
    @header.setter
    def header(self, value):
        self._header = value
    #data property
    @property
    def data(self):
        if(self._data == None):
            raise MAVlinkPlugException('No data available')
        else:
            return self._data
    @data.setter
    def data(self, value):
        #TODO : add check on this setter
        self._data = value
    def pack(self):
        return self._header.pack() + self._data
        

