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

import mavlinkplug.Message
from mavlinkplug.Base import MAVLinkPlugZmqBase, in_thread
from mavlinkplug.Exception import MAVlinkPlugException

from pymavlink import mavutil
import zmq


class MAVlinkPlugConnection(MAVLinkPlugModBase):
    def __init__(self, module_info, *argv, **kwargs):
        super(MAVLINK_connection, self).__init__()
        self._zmq_context = zmq.Context()
        self._zmq_sock_out , self._zmq_sock_in, self._ident = module_info
        self.isDaemon = False
        self._argv = argv
        self._kwargs = kwargs
        self._mavh = None
        self._in_msg = 0
        self._ok_msg = 0
        self._out_msg = 0
    def mav_handle(self):
        return self._mavh
    def try_connection(self):
        self._mavh = None
        logging.info(('MAVLINK connection {0} initialising'.format(self._ident))
        while(self._run):
            try:
                self._mavh = mavutil.mavlink_connection(*self._argv,**self._kwargs)
            except:
                self._mavh = None
                sleep(1)                                #Wait 1 second until next try
            else:
                break
        logging.info('MAVLINK connection {0} acquired'.format(self._ident))
    def _mod(self):
        self._mod_1()
        self._mod_2()
    @in_thread
    def _mod_1(self):
        socket =  self._zmq_context.socket(zmq.PUB)
        socket.connect(self._zmq_sock_out)                       #New socket which publish to the bridge
        self.try_connection()
        logging.info('MAVLINK_connection {0} loop start'.format(self._ident))
        while(self._run):
            try:
                msg = self._mavh.recv_msg()                        #Blocking TBC
            except:
                logging.info('MAVLINK connection {0} lost'.format(self._ident))
                self.try_connection()
            else:
                if msg is not None:
                    d_type = msg.get_type()
                    e_string  = ZMQ_MESSAGE_BINARY + '{0} {1:.3f} {2}'.format(self._ident, msg._timestamp,msg.get_msgbuf())
                    socket.send(e_string)
                    self._in_msg += 1
                    if (d_type != 'BAD DATA' and d_type != 'BAD_DATA'):      #BAD DATA message ignored
                        self._ok_msg += 1
                        data = {}
                        data[d_type] = {}
                        for i in msg.get_fieldnames():
                            data[d_type][i]=msg.__dict__[i]
                        try:
                            json_data = dumps(data)
                        except:
                            pass
                        else:
                            d_string = ZMQ_MESSAGE_JSON + '{0} {1:.3f} {2}'.format(self._ident, msg._timestamp , json_data)
                            self._out_msg += 1
                            socket.send(d_string)
        logging.info('MAVLINK_connection {0} loop stop'.format(self._ident))
        socket.close()
    def _mod_2(self):
        socket =  self._zmq_context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, mavlinkplug.Message.DEF_PACK(mavlinkplug.Message.MSG_PLUG_DEST_TYPE_ALL))
        socket.setsockopt(zmq.SUBSCRIBE, mavlinkplug.Message.DEF_PACK(self._ident))
        socket.connect(self._zmq_sock_in)
        while(self._run):
            _mavlinkplug_message = MAVlinkPlugMessage(msg = socket.recv())
            if(_mavlinkplug_message.get_type() == mavlinkplug.Message.MSG_PLUG_TYPE_KILL):
                self.stop()
            elif(_mavlinkplug_message.get_type() == mavlinkplug.Message.MSG_PLUG_TYPE_MAV_MSG):
                self.write(_mavlinkplug_message.get_data())
            elif(_mavlinkplug_message.get_type() == mavlinkplug.Message.MSG_PLUG_TYPE_MAV_COMMAND):
                self.mavlink_command(_mavlinkplug_message.get_data())
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
                elif(cmd == 'WP_REQUEST'):
                    self._mavh.waypoint_request_send(param['seq'])
                    logging_string = 'Launch waypoint_request_send({0}) command'.format(param['seq'])
                elif(cmd == 'SET_HIL_ARM'):
                    self._mavh.mav.command_long_send(self._mavh.target_system,
                            self._mavh.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 4,
                            mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED |
                            mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED,
                            0, 0, 0, 0, 0, 0)
                    logging_string = 'Launch set hil & arm command'
                logging.info('Ident: {0} '.format(self._ident) + logging_string)
                return True
            except Exception as e:
                print(e)
                logging.debug('Mavlink Command exception occured : {0}'.format(str(e)))
                return False
    def write(self, data):
        if(self._mavh != None):
            try:
                self._mavh.write(data)
                return True
            except: 
                pass
        return False
    def stop(self)
        self._run == False
    def info(self):
        return {'ident' :  self._ident, 'argv': self._argv, 'kwargs': self._kwargs, 'msg_stats': {'in_msg': self._in_msg, 'ok_msg': self._ok_msg, 'out_msg': self._out_msg}}
    
