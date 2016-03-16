#!/usr/bin/env python
# -*- coding: utf-8 -*-

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
from time import sleep, time

#External Module
from pymavlink import mavutil
import zmq

#Internal Module
import mavlinkplug.Message
from mavlinkplug.Base import ModBase, in_thread
from mavlinkplug.Exception import Exception
from serial.serialutil import SerialException

class MavConnection(ModBase):
    def __init__(self, module_info, *argv, **kwargs):
        super(MavConnection, self).__init__()
        self._zmq_context = zmq.Context()
        self._zmq_sock_out , self._zmq_sock_in, self._ident = module_info
        self._stream2Plug = None
        self._streamFromPlug = None
        self._argv = argv
        self._kwargs = kwargs
        self._mavh = None
        self._in_msg = 0
        self._name = 'MavConnection_' + str(self._ident)

    def try_connection(self):
        self._mavh = None
        self._logging('Connection initializing')
        while(self._run):
            try:
                self._mavh = mavutil.mavlink_connection(*self._argv, dialect = mavlinkplug._MAVLINKPLUG_DIALECT,**self._kwargs)
            except SerialException as e:
                self._mavh = None
                sleep(1)                                #Wait 1 second until next try
            else:
                break
        self._logging('Connection acquired')
    def _mod(self):
        self._mod_1()
        self._mod_2()
    @in_thread()
    def _mod_1(self):
        self._stream2Plug =  self._zmq_context.socket(zmq.PUB)
        self._stream2Plug.connect(self._zmq_sock_out)                       #New socket which publish to the bridge
        self.try_connection()
        self._logging('Loop start')

        _header = mavlinkplug.Message.Header().build_from(mavlinkplug.Message.DESTINATION.ALL.value,
                                                          self._ident,
                                                          mavlinkplug.Message.TYPE.MAV_MSG.value,
                                                          1L)
        _data = mavlinkplug.Message.MAVLinkData()
        plug_message = mavlinkplug.Message.Message().build_from(_header, _data)
        time_idle = None
        while(self._run):
            try:
                msg = self._mavh.recv_msg()
            except(Exception) as e:
                self._logging('Reinitializing connection (error)')
                self._mavh.close()
                self.try_connection()
            else:
                if msg is not None:
                    time_idle = None
                    self._in_msg += 1
                    try:
                        plug_message.header.timestamp = long(time())
                        plug_message.data.value = msg.get_msgbuf()
                    except(Exception) as e :
                        self._logging(e.message, type = 'DEBUG')
                    else:
                        self._stream2Plug.send(plug_message.packed)
                else:
                    if(time_idle == None):
                        time_idle = time()
                        log_front = 1.0
                    elif(time() - time_idle > 5.0):
                        self._logging('Reinitializing connection (idle)')
                        self._mavh.close()
                        self.try_connection()
                        time_idle = None
                    else:
                        if(time() - time_idle > log_front):
                            self._logging('Connection idle for {0:.2f} second(s)'.format(time() - time_idle))
                            log_front = log_front + 1.0
        del(plug_message)
        self._logging('Loop stop')
        self._stream2Plug.close()
    @in_thread()
    def _mod_2(self):
        self._streamFromPlug =  self._zmq_context.socket(zmq.SUB)
        self._streamFromPlug.setsockopt(zmq.SUBSCRIBE, mavlinkplug.Message.DESTINATION.ALL.p_value)
        self._streamFromPlug.setsockopt(zmq.SUBSCRIBE, mavlinkplug.Message.integer_pack(self._ident))
        self._streamFromPlug.connect(self._zmq_sock_in)
        while(self._run):
            _mavlinkplug_message = mavlinkplug.Message.Message().unpack_from(self._streamFromPlug.recv())
            #Do not process message from this module
            if(_mavlinkplug_message.header.source == self._ident):
                continue
            elif(_mavlinkplug_message.header.type == mavlinkplug.Message.TYPE.KILL.value):
                self.stop()
            elif(_mavlinkplug_message.header.type == mavlinkplug.Message.TYPE.MAV_MSG.value and _mavlinkplug_message.header.destination == self._ident):
                self.write(_mavlinkplug_message.data.packed)
            elif(_mavlinkplug_message.header.type == mavlinkplug.Message.TYPE.RAW.value and _mavlinkplug_message.header.destination == self._ident):
                self.write(_mavlinkplug_message.data.packed)
            elif(_mavlinkplug_message.header.type == mavlinkplug.Message.TYPE.MAV_COMMAND.value and _mavlinkplug_message.header.destination == self._ident):
                self.mavlink_command(_mavlinkplug_message.data.packed)
            del(_mavlinkplug_message)
        self._streamFromPlug.close()
    def mavlink_command(self, cmd):
        if(self._mavh == None):
            return False
        else :
            try:
                logging_string = '{0} : Not yet implemented'.format(cmd)
                if (cmd == 'RESET'):
                    self._mavh.reboot_autopilot()
                    logging_string = 'Launch reboot_autopilot command'
                elif (cmd == 'LOITER_MODE'):
                    self._mavh.set_mode_loiter()
                    logging_string = 'Launch set_mode_loiter command'
                elif (cmd == 'RTL_MODE'):
                    self._mavh.set_mode_rtl()
                    logging_string = 'Launch set_mode_rtl command'
                elif (cmd == 'MISSION_MODE'):
                    self._mavh.set_mode_auto()
                    logging_string = 'Launch set_mode_auto command'
                elif(cmd == 'WP_LIST_REQUEST'):
                    self._mavh.waypoint_request_list_send()
                    logging_string = 'Launch waypoint_request_list_send command'
                #elif(cmd == 'WP_REQUEST'):
                #    self._mavh.waypoint_request_send(param["seq"])
                #    logging_string = 'Launch waypoint_request_send({0}) command'.format(param["seq"])
                elif(cmd == 'SET_HIL_ARM'):
                    self._mavh.mav.command_long_send(self._mavh.target_system,
                            self._mavh.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 4,
                            mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED |
                            mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED,
                            0, 0, 0, 0, 0, 0)
                    self._mavh.mav.param_set_send(self._mavh.target_system, self._mavh.target_component, 'HIL_MODE', 1.0, 2) #Patch for apm
                    logging_string = 'Launch set hil & arm command'
                self._logging(logging_string)
                return True
            except Exception as e:
                self._logging(e, type='DEBUG')
                self._logging('Mavlink Command exception occured : {0}'.format(str(e)), type='DEBUG')
                return False
    def write(self, data):
        if(self._mavh != None):
            try:
                self._mavh.write(data)
                return True
            except: 
                pass
        return False
    def stop(self):
        self._run == False
    def _logging(self, msg, type='INFO'):
         logging_message = mavlinkplug.Message.LogData.build_full_message_from( mavlinkplug.Message.DESTINATION.ALL.value,
                                                                                self._ident,
                                                                                long(time()),
                                                                                type+': '+ msg
                                                                                )
         self._stream2Plug.send(logging_message.packed)

