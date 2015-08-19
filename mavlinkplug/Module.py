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
import logging
from time import sleep, time

#External Module
from pymavlink import mavutil
import zmq
import socket
import multiprocessing

#Internal Module
import  mavlinkplug.Message
from mavlinkplug.Base import MAVLinkPlugModBase, MAVLinkPlugZmqBase, in_thread
from mavlinkplug.Exception import MAVlinkPlugException



class MAVlinkPlugConnection(MAVLinkPlugModBase):
    def __init__(self, module_info, *argv, **kwargs):
        super(MAVlinkPlugConnection, self).__init__()
        self._name = 'MAVlinkPlugConnection'
        self._zmq_context = zmq.Context()
        self._zmq_sock_out , self._zmq_sock_in, self._ident = module_info
        self._argv = argv
        self._kwargs = kwargs
        self._mavh = None
        self._in_msg = 0
    def try_connection(self):
        self._mavh = None
        logging.info('MAVlinkPlugConnection {0} connection initializing'.format(self._ident))
        while(self._run):
            try:
                self._mavh = mavutil.mavlink_connection(*self._argv,**self._kwargs)
            except Exception as e:
                self._mavh = None
                sleep(1)                                #Wait 1 second until next try
            else:
                break
        logging.info('MAVlinkPlugConnection {0} connection acquired'.format(self._ident))
    def _mod(self):
        self._mod_1()
        self._mod_2()
    @in_thread(isDaemon = True)
    def _mod_1(self):
        socket =  self._zmq_context.socket(zmq.PUB)
        socket.connect(self._zmq_sock_out)                       #New socket which publish to the bridge
        self.try_connection()
        logging.info('MAVlinkPlugConnection {0} loop start'.format(self._ident))

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
            except :
                logging.info('MAVlinkPlugConnection {0} reinitializing connection (error)'.format(self._ident))
                self._mavh.close()
                self.try_connection()
            else:
                if msg is not None:
                    time_idle = None
                    logging.debug(msg)
                    self._in_msg += 1
                    try:
                        plug_message.header.timestamp = long(msg._timestamp*1000)
                        plug_message.data.value = msg.get_msgbuf()
                    except(MAVlinkPlugException) as e :
                        logging.debug(e)
                    else:
                        socket.send(plug_message.packed)
                else:
                    if(time_idle == None):
                        time_idle = time()
                        log_front = 1.0
                    elif(time() - time_idle > 5.0):
                        logging.info('MAVlinkPlugConnection {0} reinitializing connection (idle)'.format(self._ident))
                        self._mavh.close()
                        self.try_connection()
                        time_idle = None
                    else:
                        if(time() - time_idle > log_front):
                            logging.info('MAVlinkPlugConnection {0} connection idle for {1:.2f} second(s)'.format(self._ident, time() - time_idle))
                            log_front = log_front + 1.0
        del(plug_message)
        logging.info('MAVlinkPlugConnection {0} loop stop'.format(self._ident))
        socket.close()
    @in_thread(isDaemon = True)
    def _mod_2(self):
        socket =  self._zmq_context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, mavlinkplug.Message.DESTINATION.ALL.p_value)
        socket.setsockopt(zmq.SUBSCRIBE, mavlinkplug.Message.integer_pack(self._ident))
        socket.connect(self._zmq_sock_in)
        while(self._run):
            _mavlinkplug_message = mavlinkplug.Message.Message().unpack_from(socket.recv())
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
                    logging_string = 'Launch set hil & arm command'
                logging.info('Ident: {0} '.format(self._ident) + logging_string)
                return True
            except Exception as e:
                logging.info(e)
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
    def stop(self):
        self._run == False
    def info(self):
        return {'ident' :  self._ident, 'argv': self._argv, 'kwargs': self._kwargs, 'msg_stats': {'in_msg': self._in_msg, 'ok_msg': self._ok_msg, 'out_msg': self._out_msg}}
    
class MAVlinkPlugFileWriter(MAVLinkPlugZmqBase):
    def __init__(self, module_info, file):
        super(MAVlinkPlugFileWriter, self).__init__()
        self._addr_to_plug, self._addr_from_plug, self._ident =  module_info
        self._file_name = file
        self._file_descriptor = None
        self.daemon = True
        self._subscribe = ['']
        logging.info('MAVlinkPlugFileWriter Initializing')
    def setup(self):
        super(MAVlinkPlugFileWriter,self).setup()
        #Define stream listening from plug
        self.stream(zmq.SUB, self._addr_from_plug, bind = False, callback = self._plug_2_file, subscribe = self._subscribe)
        self._file_descriptor = open(self._file_name, 'w', 500)
    def _plug_2_file(self, p_msg):
        plug_msg = mavlinkplug.Message.Message().unpack_from(p_msg[0])
        string_to_write = '{0}\t{1}\t{2}\t{3}\t{4}'.format(plug_msg.header.timestamp, plug_msg.header.destination, plug_msg.header.source, plug_msg.header.type, plug_msg.data.value)
        print(string_to_write, file = self._file_descriptor)
    def stop(self):
        super(MAVlinkPlugFileWriter, self).stop()
        self._file_descriptor.close()

class MAVLinkPlugTCPConnection(multiprocessing.Process):
    def __init__(self, module_info, tcp_tuple, mav_connection_ident):
        super(MAVLinkPlugTCPConnection,self).__init__()
        self._addr_to_plug, self._addr_from_plug, self._ident =  module_info
        self._tcp_tuple = tcp_tuple
        self._mav_connection_ident = mav_connection_ident
        self.daemon = True
        self._run = True
        self._subscribe = ['']
        logging.info('MAVLinkPlugTCPConnection Initializing')
    def run(self):
        self._connection_activated = False
        self._connection_h = None
        self._zmq_context = zmq.Context()
        #Create TCP socket
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP
        self._socket.bind(self._tcp_tuple)
        while(self._run):
            if(self._connection_activated == False):
                if(self._connection_h != None):
                    self._connection_h.close()
                logging.info('MAVLinkPlugTCPConnection {0} waiting '.format(self._ident))
                self._socket.listen(1)
                self._connection_h, self._connection_address = self._socket.accept()
                self._connection_activated = True
                logging.info('MAVLinkPlugTCPConnection {0} : connected to {1}'.format(self._ident, self._connection_address))
                #Launch threaded functions
                self._ZMQ_2_UDP()
                self._UDP_2_ZMQ()
            sleep(1)
    @mavlinkplug.Base.in_thread(True)
    def _ZMQ_2_UDP(self):
        #Create ZMQ socket
        _zmq_from_plug = self._zmq_context.socket(zmq.SUB)
        for opt in self._subscribe:
                _zmq_from_plug.setsockopt(zmq.SUBSCRIBE, opt)
        _zmq_from_plug.connect(self._addr_from_plug)
        while(self._connection_activated):
            mavlinkplug_message = mavlinkplug.Message.Message().unpack_from(_zmq_from_plug.recv())
            if(mavlinkplug_message.header.type == mavlinkplug.Message.TYPE.MAV_MSG.value and mavlinkplug_message.header.source == self._mav_connection_ident):
                try:
                    self._connection_h.send(mavlinkplug_message.data.packed)
                except:
                    self._connection_activated = False
    @mavlinkplug.Base.in_thread(True)
    def _UDP_2_ZMQ(self):
        #Create ZMQ socket
        _zmq_to_plug = self._zmq_context.socket(zmq.PUB)
        _zmq_to_plug.connect(self._addr_to_plug)
        _header = mavlinkplug.Message.Header().build_from(self._mav_connection_ident,
                                                          self._ident,
                                                          mavlinkplug.Message.TYPE.RAW.value,
                                                          1L)
        _mavlink_plug_message = mavlinkplug.Message.Message()
        _mavlink_plug_message.header = _header
        while(self._connection_activated):
            try:
                _in_data = self._connection_h.recv(1024)
            except:
                self._connection_activated = False
            else:
                _mavlink_plug_message.header.timestamp = long(time()*1000)
                try:
                    _mavlink_plug_message.data =  mavlinkplug.Message.MAVLinkData(_in_data)
                except(MAVlinkPlugException) :
                    _mavlink_plug_message.data =  mavlinkplug.Message.RawData(_in_data)
                _zmq_to_plug.send(_mavlink_plug_message.packed)
    def stop(self):
        logging.info('MAVLinkPlugTCPConnection {0} Stopping'.format(self._ident))
        self.terminate()
    def insert_ident(self, string):
        return self._name + ' {0} : '.format(self._ident)