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

from __future__ import print_function
import zmq, logging
from time import sleep, time
from mavlinkplug.Base import MAVLinkPlugZmqBase
import  mavlinkplug.Message


class MAVLinkPlugHil(MAVLinkPlugZmqBase):
    def __init__(self, module_info, mavlink_connection_ident, Aircraft_Type_cls):
        super(MAVLinkPlugHil, self).__init__()
        self._mavlink_connection_ident = mavlink_connection_ident
        self._addr_to_plug, self._addr_from_plug, self._ident =  module_info
        self._addr_to_FL = 'tcp://127.0.0.1:45063'
        self._addr_from_FL = 'tcp://127.0.0.1:45064'
        self._Aircraft_Type_cls = Aircraft_Type_cls
        self.daemon = True
        self._default_subscribe.append(mavlinkplug.Message.integer_pack(self._ident))
        self._dumb_header = mavlinkplug.Message.mavlink.MAVLink_header(0)
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
        '''
        Get message from plug and pass it to FL
        Call a class method from _Aircraft_Type_cls to adapt the message to send
        :param msg: mavlink plug message from plug
        :return: Nothing
        '''
        _msg = msg[0] #get the first (and only) part of the message
        mavlinkplug_message = mavlinkplug.Message.Message()
        mavlinkplug_message.unpack_from(_msg)
        if(mavlinkplug_message.header.type == mavlinkplug.Message.TYPE.MAV_MSG.value and mavlinkplug_message.header.source != self._ident):
            data_2_FL = self._Aircraft_Type_cls.mav_2_FL(mavlinkplug_message.data.value)
            if(data_2_FL != None):
                #Stringify
                data_2_FL = map(str,data_2_FL)
                msg_2_FL = ' '.join(data_2_FL)
                #Send to Flight Loop
                self._stream_to_FL.send(msg_2_FL)
        del(mavlinkplug_message)
    def _FL_2_plug(self, msg):
        '''
        Get message from FL and pass it to plug
        Call a class method from _Aircraft_Type_cls to adapt the message to send
        :param msg: ZMQ message from FL
        :return: Nothing
        '''

        _msg = msg[0] #get the first (and only) part of the message

        _data_2_plug = self._Aircraft_Type_cls.FL_2_mav(_msg)

        #Mavlink Message Creation
        _mav_message = mavlinkplug.Message.mavlink.MAVLink_hil_sensor_message(*_data_2_plug).pack(self._dumb_header)

        #MavlinkPlug Message Creation
        _header = mavlinkplug.Message.Header().build_from(self._mavlink_connection_ident,
                                                          self._ident,
                                                          mavlinkplug.Message.TYPE.MAV_MSG.value,
                                                          long(time())
                                                          )
        _data = mavlinkplug.Message.MAVLinkData().build_from(_mav_message)
        _mavlink_plug_message = mavlinkplug.Message.Message().build_from(_header,_data)

        #Sending MavlinkPLug Message
        self._stream_to_plug.send(_mavlink_plug_message.packed)

        del(_mavlink_plug_message)