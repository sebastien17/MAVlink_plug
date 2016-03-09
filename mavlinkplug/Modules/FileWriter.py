#!/usr/bin/env python
# -*- coding: utf-8 -*-

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
from time import time
#External Module
import zmq
#Internal Module
import  mavlinkplug.Message
from mavlinkplug.Base import ZmqBase


class FileWriter(ZmqBase):
    def __init__(self, module_info, file, name = None):
        super(FileWriter, self).__init__()
        self._addr_to_plug, self._addr_from_plug, self._ident = module_info
        self._stream2Plug = None
        self._streamFromPlug = None
        self._file_name = file
        self._file_descriptor = None
        self._subscribe = ['']
        if(name == None ):
            self._name = 'FileWriter_' + str(self._ident)
        else:
            self._name = name
    def setup(self):
        super(FileWriter,self).setup()
        #Define stream listening from plug
        self._streamFromPlug = self.stream(zmq.SUB, self._addr_from_plug, bind = False, callback = self._plug_2_file, subscribe = self._subscribe)
        self._stream2Plug  = self.stream(zmq.PUB, self._addr_to_plug, bind = False)
        self._file_descriptor = open(self._file_name, 'w', 100)
        self._logging('Initializing')
    def _plug_2_file(self, p_msg):
        plug_msg = mavlinkplug.Message.Message().unpack_from(p_msg[0])
        string_to_write = '{0}\t{1}\t{2}\t{3}\t{4}'.format(plug_msg.header.timestamp, plug_msg.header.destination, plug_msg.header.source, plug_msg.header.type, plug_msg.data.value)
        print(string_to_write, file = self._file_descriptor)
    def stop(self):
        super(FileWriter, self).stop()
        self._file_descriptor.close()
        self._logging('Closing')
    def _logging(self, msg, type = 'INFO'):
         logging_message = mavlinkplug.Message.LogData.build_full_message_from( 0,
                                                                                self._ident,
                                                                                long(time()),
                                                                                type+': '+ msg
                                                                                )
         self._stream2Plug.send(logging_message.packed)
