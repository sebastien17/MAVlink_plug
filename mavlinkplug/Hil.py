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

#Core Module
from __future__ import print_function
import zmq, logging
from time import sleep

#Internal Module
from mavlinkplug.Base import MAVLinkPlugZmqBase
import  mavlinkplug.Message


class MAVLinkPlugHil(MAVLinkPlugZmqBase):
    def __init__(self, module_info, mavlink_connection_ident, Aircraft_Type_cls):
        super(MAVLinkPlugHil, self).__init__()
        self._addr_to_plug, self._addr_from_plug, self._ident =  module_info
        self._addr_to_FL = 'tcp://127.0.0.1:45063'
        self._addr_from_FL = 'tcp://127.0.0.1:45064'
        self._Aircraft_Type_cls = Aircraft_Type_cls
        self.daemon = True
        self._default_subscribe.append(mavlinkplug.Message.DEF_PACK(self._ident))
    def setup(self):
        super(MAVLinkPlugHil,self).setup()
        #Define stream listening from plug
        self.stream(zmq.SUB, self._addr_from_plug, bind = False, callback = self._plug_2_FL)
        #Define stream publishing to FL
        self._stream_to_FL  = self.stream(zmq.PUB, self._addr_to_FL)
        #Define stream listening from FL
        self.stream(zmq.SUB, self._addr_from_FL, callback = self._FL_2_plug, subscribe = [b''])
        #Define stream publishing to plug
        self._stream_to_plug  = self.stream(zmq.PUB, self._addr_to_plug, bind = False)
    def hardware_initialize(self):
        # while(self._Mav_inst.mav_handle() == None):
            # time.sleep(1)
        # self._Mav_inst.mavlink_command('SET_HIL_ARM')
        #TODO : add check
        # time.sleep(2)
        # self._Mav_inst.mavlink_command('RESET')
        #TODO : add check
        pass
    def FL_initialize(self):
        logging.info('Initializing Flight Loop')
        aircraft = self._Aircraft_Type_cls(zmq_in = self._addr_to_FL, zmq_out = self._addr_from_FL)
        aircraft.start()
    def _plug_2_FL(self, msg):
        #get the first (and only) part of the message
        _msg = msg[0]
        mavlinkplug_message = mavlinkplug.Message.MAVlinkPlugMessage(_msg)
        if(mavlinkplug_message.header.type == mavlinkplug.Message.MSG_PLUG_TYPE_MAV_MSG):
            data_2_FL = self._Aircraft_Type_cls.mav_2_FL(mavlinkplug_message.data)
            if(data_2_FL != None):
                #Stringify
                data_2_FL = map(str,data_2_FL)
                msg_2_FL = ' '.join(data_2_FL)
                #Send to Flight Loop
                self._stream_to_FL.send(msg_2_FL)
        del(mavlinkplug_message)
    def _FL_2_plug(self, msg):
        _msg = msg[0]
        data_2_plug = self._Aircraft_Type_cls.FL_2_mav(_msg)