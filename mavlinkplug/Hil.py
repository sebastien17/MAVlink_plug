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
    def __init__(self, module_info, mavlink_connection_ident, Aircraft_Type_cls, hil_sensor=True, quaternion=True):
        super(MAVLinkPlugHil, self).__init__()
        self._mavlink_connection_ident = mavlink_connection_ident
        self._addr_to_plug, self._addr_from_plug, self._ident =  module_info
        self._addr_to_FL = 'tcp://127.0.0.1:45063'
        self._addr_from_FL = 'tcp://127.0.0.1:45064'
        self._Aircraft_Type_cls = Aircraft_Type_cls
        self.daemon = False
        self._default_subscribe.append(mavlinkplug.Message.integer_pack(self._ident))
        self._dumb_header = mavlinkplug.Message.mavlink.MAVLink_header(0)
        self._FL_loop_p = None
        self._phase = 0
        self._hb_count = 0
        self._hil_sensor = hil_sensor
        self._quaternion = quaternion

    def setup(self):
        super(MAVLinkPlugHil,self).setup()
        # Initializing message callback
        # Define stream listening from plug
        self.stream(zmq.SUB, self._addr_from_plug, bind = False, callback = self._plug_2_FL)
        #Define stream publishing to FL
        self._stream_to_FL  = self.stream(zmq.PUB, self._addr_to_FL)
        #Define stream listening from FL
        self.stream(zmq.SUB, self._addr_from_FL, callback = self._FL_2_plug, subscribe = [b''])
        #Define stream publishing to plug
        self._stream_to_plug  = self.stream(zmq.PUB, self._addr_to_plug, bind = False)

    def _get_MAVlink_Plug_Message(selfself, msg):
        _msg = msg[0] #get the first (and only) part of the message
        _message = mavlinkplug.Message.Message()
        _message.unpack_from(_msg)
        return _message

    def _plug_2_FL(self, msg):
        _message = self._get_MAVlink_Plug_Message(msg)
        if(_message.header.source == self._mavlink_connection_ident):
            if(self._phase == 17):  #RUN
                self._phase_RUN(_message)
            elif(self._phase == 13):
                self._phase_WAIT_HB(_message)
            elif(self._phase == 11):
                self._phase_RESET()
            elif(self._phase == 7):
                self._phase_WAIT_FOR_ALIVE(_message)
            elif(self._phase == 0):
                self._phase_INIT()

    def _phase_INIT(self):
        logging.info('INIT phase start')
        self._FL_loop_p = self._Aircraft_Type_cls(zmq_in = self._addr_to_FL, zmq_out = self._addr_from_FL, lat = 43.6042600, lon = 1.4436700) # Toulouse, France
        logging.info('INIT phase end')
        self._phase = 7 #Go to WAIT_FOR_ALIVE

    def _phase_WAIT_FOR_ALIVE(self,msg):
        if(self._hb_count == 0):
            logging.info('WAIT_FOR_ALIVE phase start')
        if(msg.header.type == mavlinkplug.Message.TYPE.MAV_MSG.value and msg.data.value.get_type() == 'HEARTBEAT'):
            self._hb_count += 1
        if(self._hb_count == 10):
            self._hb_count = 0
            self._phase = 11 #Go to RESET
            logging.info('WAIT_FOR_ALIVE phase end')

    def _phase_RESET(self):
        logging.info('RESET phase start')

        #MavlinkPlug Command Message Creation
        _header = mavlinkplug.Message.Header().build_from(self._mavlink_connection_ident,self._ident,mavlinkplug.Message.TYPE.MAV_COMMAND.value,long(time()))
        _data = mavlinkplug.Message.MavCommandData().build_from('SET_HIL_ARM')
        _mavlink_plug_command_message = mavlinkplug.Message.Message().build_from(_header,_data)
        self._stream_to_plug.send(_mavlink_plug_command_message.packed)

        sleep(1)

        #MavlinkPlug Command Message Creation
        _data = mavlinkplug.Message.MavCommandData().build_from('RESET')
        _mavlink_plug_command_message = mavlinkplug.Message.Message().build_from(_header,_data)
        self._stream_to_plug.send(_mavlink_plug_command_message.packed)

        logging.info('RESET phase end')
        self._phase = 13 #Go to WAIT_FOR_ALIVE

    def _phase_WAIT_HB(self,msg):
        if(self._hb_count == 0):
            logging.info('WAIT_HB phase start')
        if(msg.header.type == mavlinkplug.Message.TYPE.MAV_MSG.value and msg.data.value.get_type() == 'HEARTBEAT'):
            self._hb_count += 1
        if(self._hb_count == 5):
            self._hb_count = 0
            self._FL_loop_p.start()
            self._phase = 17 #Go to RESET

            logging.info('WAIT_HB phase end')

    def _phase_RUN(self,msg):
        '''
        Get message from plug and pass it to FL
        Call a class method from _Aircraft_Type_cls to adapt the message to send
        :param msg: mavlink plug message from plug
        :return: Nothing
        '''

        if(msg.header.type == mavlinkplug.Message.TYPE.MAV_MSG.value):
            data_2_FL = self._Aircraft_Type_cls.mav_2_FL(msg.data.value)
            if(data_2_FL != None):
                #Stringify
                data_2_FL = map(str,data_2_FL)
                msg_2_FL = ' '.join(data_2_FL)
                #Send to Flight Loop
                print(msg_2_FL)
                self._stream_to_FL.send(msg_2_FL)

    def _FL_2_plug(self, msg):
        '''
        Get message from FL and pass it to plug
        Call a class method from _Aircraft_Type_cls to adapt the message to send and send the mavlink message through
        Plug Message
        :param msg: ZMQ message from FL
        :return: Nothing
        '''

        msg = msg[0]  # get the first (and only) part of the message

        if(self._hil_sensor):
            #HIL sensor Mavlink Message Creation
            parameters = self._Aircraft_Type_cls.FL_2_mav_sensor(msg)
            mav_message = mavlinkplug.Message.mavlink.MAVLink_hil_sensor_message(*parameters).pack(self._dumb_header)
        else:
            if(self._quaternion):
                # HIL State Quaternion Mavlink Message Creation
                # 'time_usec', 'attitude_quaternion', 'rollspeed', 'pitchspeed', 'yawspeed',
                # 'lat', 'lon', 'alt', 'vx', 'vy', 'vz', 'ind_airspeed', 'true_airspeed', 'xacc', 'yacc', 'zacc']
                parameters = self._Aircraft_Type_cls.FL_2_mav_state_quaternion(msg)
                mav_message = mavlinkplug.Message.mavlink.MAVLink_hil_state_quaternion_message(*parameters).pack(self._dumb_header)
            else:
                # HIL State Mavlink Message Creation
                # 'time_usec', 'roll', 'pitch', 'yaw', 'rollspeed', 'pitchspeed', 'yawspeed',
                # 'lat', 'lon', 'alt', 'vx', 'vy', 'vz', 'xacc', 'yacc', 'zacc'
                parameters = self._Aircraft_Type_cls.FL_2_mav_state(msg)
                try:
                    mav_message = mavlinkplug.Message.mavlink.MAVLink_hil_state_message(*parameters).pack(self._dumb_header)
                except Exception as e:
                    print(e.message)


        try:
            #MavlinkPlug Message Creation
            mavlink_plug_message = mavlinkplug.Message.MAVLinkData.build_full_message_from(
                                                                                            self._mavlink_connection_ident,
                                                                                            self._ident,
                                                                                            long(time()),
                                                                                            mav_message
                                                                                            )
            #Sending MavlinkPLug Message
            self._stream_to_plug.send(mavlink_plug_message.packed)
        except Exception as e:
             print(e.message)