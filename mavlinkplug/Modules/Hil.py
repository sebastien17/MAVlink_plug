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

from __future__ import print_function
import zmq, math
from time import sleep, time
from mavlinkplug.Base import ZmqBase
import  mavlinkplug.Message, mavlinkplug.Tools


class Hil(ZmqBase):
    def __init__(self, module_info, mavlink_connection_ident, Aircraft_Type_cls, hil_sensor=True, quaternion=True, name=None):
        super(Hil, self).__init__()
        self._mavlink_connection_ident = mavlink_connection_ident
        self._addr_to_plug, self._addr_from_plug, self._ident =  module_info
        self._addr_to_FL = 'tcp://127.0.0.1:45063'
        self._addr_from_FL = 'tcp://127.0.0.1:45064'
        self._Aircraft_Type_cls = Aircraft_Type_cls
        self._default_subscribe.append(mavlinkplug.Message.integer_pack(self._ident))
        self._dumb_header = mavlinkplug.Message.mavlink.MAVLink_header(0)
        self._FL_loop_p = None
        self._phase = 0
        self._hb_count = 0
        self._hil_sensor = hil_sensor
        self._quaternion = quaternion
        self._thermal_wind_NED = (0.0, 0.0, 0.0)
        self._thermal_list = []
        if(name == None ):
            self._name = 'FileWriter_' + self._ident
        else:
            self._name = name
    def setup(self):
        super(Hil, self).setup()
        # Initializing message callback
        # Define stream listening from plug
        self.stream(zmq.SUB, self._addr_from_plug, bind = False, callback = self._plug_2_FL)
        #Define stream publishing to FL
        self._stream_to_FL  = self.stream(zmq.PUB, self._addr_to_FL)
        #Define stream listening from FL
        self.stream(zmq.SUB, self._addr_from_FL, callback = self._FL_2_plug, subscribe = [b''])
        #Define stream publishing to plug
        self._stream2Plug  = self.stream(zmq.PUB, self._addr_to_plug, bind = False)

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
        self._logging('INIT phase start')
        self._FL_loop_p = self._Aircraft_Type_cls(zmq_in = self._addr_to_FL, zmq_out = self._addr_from_FL, lat = 43.6042600, lon = 1.4436700) # Toulouse, France
        self._logging('INIT phase end')
        self._phase = 7 #Go to WAIT_FOR_ALIVE

    def _phase_WAIT_FOR_ALIVE(self,msg):
        if(self._hb_count == 0):
            self._logging('WAIT_FOR_ALIVE phase start')
        if(msg.header.type == mavlinkplug.Message.TYPE.MAV_MSG.value and msg.data.value.get_type() == 'HEARTBEAT'):
            self._hb_count += 1
        if(self._hb_count == 10):
            self._hb_count = 0
            self._phase = 11 #Go to RESET
            self._logging('WAIT_FOR_ALIVE phase end')

    def _phase_RESET(self):
        self._logging('RESET phase start')

        #MavlinkPlug Command Message Creation
        _header = mavlinkplug.Message.Header().build_from(self._mavlink_connection_ident,self._ident,mavlinkplug.Message.TYPE.MAV_COMMAND.value,long(time()))
        _data = mavlinkplug.Message.MavCommandData().build_from('SET_HIL_ARM')
        _mavlink_plug_command_message = mavlinkplug.Message.Message().build_from(_header,_data)
        self._stream2Plug.send(_mavlink_plug_command_message.packed)

        sleep(1)

        #MavlinkPlug Command Message Creation
        _data = mavlinkplug.Message.MavCommandData().build_from('RESET')
        _mavlink_plug_command_message = mavlinkplug.Message.Message().build_from(_header,_data)
        self._stream2Plug.send(_mavlink_plug_command_message.packed)

        self._logging('RESET phase end')
        self._phase = 13 #Go to WAIT_FOR_ALIVE

    def _phase_WAIT_HB(self,msg):
        if(self._hb_count == 0):
            self._logging('WAIT_HB phase start')
        if(msg.header.type == mavlinkplug.Message.TYPE.MAV_MSG.value and msg.data.value.get_type() == 'HEARTBEAT'):
            self._hb_count += 1
        if(self._hb_count == 5):
            self._hb_count = 0
            self._FL_loop_p.start()
            self._phase = 17 #Go to RESET

            self._logging('WAIT_HB phase end')

    def _phase_RUN(self,msg):
        '''
        Get message from plug and pass it to FL
        Call a class method from _Aircraft_Type_cls to adapt the message to send
        :param msg: mavlink plug message from plug
        :return: Nothing
        '''

        if(msg.header.type == mavlinkplug.Message.TYPE.MAV_MSG.value):
            data_2_FL = self._Aircraft_Type_cls.mav_2_FL(msg.data.value, self._thermal_wind_NED)
            if(data_2_FL != None):
                #Stringify
                data_2_FL = map(str,data_2_FL)
                msg_2_FL = ' '.join(data_2_FL)
                #Send to Flight Loop
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
            # HIL sensor Mavlink Message Creation
            # 'time_msec', 'xacc', 'yacc', 'zacc', 'xgyro', 'ygyro', 'zgyro', 'xmag', 'ymag', 'zmag',
            # 'abs_pressure in millibar', 'diff_pressure', 'pressure_alt', 'temperature', 'fields_updated'
            parameters = self._Aircraft_Type_cls.FL_2_mav_sensor(msg)
            mav_message = mavlinkplug.Message.mavlink.MAVLink_hil_sensor_message(*parameters).pack(self._dumb_header)
        else:
            if(self._quaternion):
                # HIL State Quaternion Mavlink Message Creation
                # 'time_usec', 'attitude_quaternion', 'rollspeed', 'pitchspeed', 'yawspeed',
                # 'lat', 'lon', 'alt', 'vx', 'vy', 'vz', 'ind_airspeed', 'true_airspeed', 'xacc', 'yacc', 'zacc']
                parameters = self._Aircraft_Type_cls.FL_2_mav_state_quaternion(msg)
                try:
                    mav_message = mavlinkplug.Message.mavlink.MAVLink_hil_state_quaternion_message(*parameters).pack(self._dumb_header)
                except Exception as e:
                    print(e.message)
                else:
                    deg_coordinates_tuple = (parameters[5], parameters[6])
            else:
                # HIL State Mavlink Message Creation
                # 'time_usec', 'roll', 'pitch', 'yaw', 'rollspeed', 'pitchspeed', 'yawspeed',
                # 'lat', 'lon', 'alt', 'vx', 'vy', 'vz', 'xacc', 'yacc', 'zacc'
                parameters = self._Aircraft_Type_cls.FL_2_mav_state(msg)
                try:
                    mav_message = mavlinkplug.Message.mavlink.MAVLink_hil_state_message(*parameters).pack(self._dumb_header)
                except Exception as e:
                    print(e.message)
                else:
                    self._deg_coordinates_tuple = (parameters[5], parameters[6])

        try:
            #MavlinkPlug Message Creation
            mavlink_plug_message = mavlinkplug.Message.MAVLinkData.build_full_message_from( self._mavlink_connection_ident,
                                                                                            self._ident,
                                                                                            long(time()),
                                                                                            mav_message
                                                                                            )
            #Sending MavlinkPLug Message
            self._stream2Plug.send(mavlink_plug_message.packed)

            #Energy Information Message Creation
            raw_string = "KE_PE {0} {1}".format(self._Aircraft_Type_cls.kinetic_energy(msg),self._Aircraft_Type_cls.potential_energy(msg))
            energy_message = mavlinkplug.Message.RawData.build_full_message_from(mavlinkplug.Message.DESTINATION.ALL.value,
                                                                                 self._ident,
                                                                                 long(time()),
                                                                                 raw_string)
            self._stream2Plug.send(energy_message.packed)
        except Exception as e:
             print(e.message)
        else:
            self._thermals_management(deg_coordinates_tuple)

    def add_thermal(self, deg_lat_long_tuple, amplitude, D_param):
        self._thermal_list.append(tuple(deg_lat_long_tuple, amplitude, D_param))

    def _thermals_management(self, deg_coordinates_tuple):
        self._thermal_wind_NED = (0.0, 0.0, 0.0)

        def myadd(xs,ys):
            return tuple(x + y for x, y in zip(xs, ys))

        for thermal in self._thermal_list:
            self._thermal_wind_NED = myadd(self._thermal_results(thermal), self._thermal_wind_NED)

    def _thermal_results(self, thermal):
        distance = mavlinkplug.Tools.distance_from_coordinates_degrees(thermal[0], self._deg_coordinates_tuple)
        D = thermal[2] # Thermal radius with positive vertical wind in meters : D_param
        amplitude = thermal[1]

        # Very simple model
        if (distance <= 7*D):
            result = (0.0, 0.0, -amplitude*2*math.exp(-distance/D))
        else:
            result = (0.0, 0.0, 0.0)
        return result

    def _logging(self, msg, type = 'INFO'):
         logging_message = mavlinkplug.Message.LogData.build_full_message_from( 0,
                                                                                self._ident,
                                                                                long(time()),
                                                                                type+': '+ msg
                                                                                )
         self._stream2Plug.send(logging_message.packed)