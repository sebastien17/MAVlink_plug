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

#Core import
from __future__ import print_function
import multiprocessing, logging
from json import loads
from os import path, sep
from pyfdm import fdmexec
from pyfdm.exchange import zmq_exchange
from zmq import Context
from time import time


class QuadCopter(multiprocessing.Process):
    
    _data_in = [ 
    'fcs/throttle-cmd-norm[0]',
    'fcs/throttle-cmd-norm[1]',
    'fcs/throttle-cmd-norm[2]',
    'fcs/throttle-cmd-norm[3]'
    ]

    _data_out = [
    #for HIL_GPS 
    'position/lat-gc-rad',
    'position/long-gc-rad',
    'position/h-sl-ft',
    #for HIL_SENSOR
    #Need magnetic field composant
    'accelerations/udot-ft_sec2',
    'accelerations/vdot-ft_sec2',
    'accelerations/wdot-ft_sec2',
    'velocities/p-aero-rad_sec',
    'velocities/q-aero-rad_sec',
    'velocities/r-aero-rad_sec',
    'atmosphere/pressure-altitude',
    'attitude/heading-true-rad',
    'sensors/magnetometer/X/output',
    'sensors/magnetometer/Y/output',
    'sensors/magnetometer/Z/output'
    ]
    
    JSBSIM_DEFAULT_PATH = path.dirname(__file__) + sep + 'data' + sep
    MIN_SERVO_PPM = 950
    MAX_SERVO_PPM = 1800

    def __init__(self, zmq_context = None, zmq_in = None, zmq_out = None, daemon = True, lat = 0, long = 0, terrain_elev_ft = 0, h_agl_ft = 1, fdm_model = 'arducopter', jsbsim_root = None, dt = 1.0/100):
        super(QuadCopter,self).__init__()
        self._zmq_context = zmq_context
        self._zmq_in = zmq_in
        self._zmq_out = zmq_out
        self.daemon = daemon
        self._lat_rad = lat
        self._long_rad = long
        self._terrain_elev_ft = terrain_elev_ft
        self._h_agl_ft = h_agl_ft
        self._fdm_model = fdm_model
        if(jsbsim_root != None):
            self._jsbsim_root = jsbsim_root
        else:
            self._jsbsim_root = self.JSBSIM_DEFAULT_PATH
        self._dt = dt
    def setup(self):
        #Flight Loop setup
        self._fdm = fdmexec.FGFDMExec(root_dir=self._jsbsim_root)
        self._fdm.load_model(self._fdm_model)
        #Initial Conditions
        self._fdm.set_property_value("ic/lat-gc-rad", self._lat_rad)
        self._fdm.set_property_value("ic/long-gc-rad",self._long_rad)
        self._fdm.set_property_value("ic/terrain-elevation-ft", self._terrain_elev_ft)
        self._fdm.set_property_value("ic/h-agl-ft",self._h_agl_ft)
        #Fdm Trim
        self._fdm.do_trim(1)
        # Zmq setup
        if(self._zmq_context == None):
            self._zmq_context = Context()
        self._zmq_tool = zmq_exchange(self._data_in,self._data_out ,self._zmq_in,self._zmq_out)
        self._fdm.exchange_register(self._zmq_tool)
    def run(self):
        self.setup()
        self._fdm.realtime(self._dt)
    def stop(self):
        self.terminate()
    @classmethod
    def servo_norm(cls, value):
        '''
        Normalize servo output data
        :param value: raw value of servo output
        :return:    normalized value of servo output
        '''
        return (value - cls.MIN_SERVO_PPM)/(cls.MAX_SERVO_PPM - cls.MIN_SERVO_PPM)
    @classmethod
    def mav_2_FL(cls, mavlink_msg):
        '''
        Adapting parameters from SERVO_OUTPUT_RAW type MAVlink message to FL (JSBsim)
        Class method for multiprocess purpose
        :param mavlink_msg: mavlink message
        :return: tuples including servo raw values or None if message not
        '''

        if(mavlink_msg.get_type() == 'SERVO_OUTPUT_RAW'):
            return (
                    cls.servo_norm(float(mavlink_msg.__dict__['servo1_raw'])),
                    cls.servo_norm(float(mavlink_msg.__dict__['servo2_raw'])),
                    cls.servo_norm(float(mavlink_msg.__dict__['servo3_raw'])),
                    cls.servo_norm(float(mavlink_msg.__dict__['servo4_raw']))
                    )
        return None
    @classmethod
    def FL_2_mav(cls, string):
        '''
        :param string: ZMQ message string including  _data_out values (Human Readable)
        :return: mavlink message to send
        '''
                #data_2_plug format :
        #        [
        #        #for HIL_GPS
        # 0       'position/lat-gc-rad',
        # 1       'position/long-gc-rad',
        # 2       'position/h-sl-ft',
        #        #for HIL_SENSOR
        #        #Need magnetic field composant
        # 3       'accelerations/udot-ft_sec2',
        # 4       'accelerations/vdot-ft_sec2',
        # 5       'accelerations/wdot-ft_sec2',
        # 6       'velocities/p-aero-rad_sec',
        # 7       'velocities/q-aero-rad_sec',
        # 8       'velocities/r-aero-rad_sec',
        # 9       'atmosphere/pressure-altitude',
        # 10      'attitude/heading-true-rad',
        # 11      'sensors/magnetometer/X/output',
        # 12      'sensors/magnetometer/Y/output',
        # 13      'sensors/magnetometer/Z/output'
        #        ]

        _temp = string.split(" ")

        #Data treatment
        _messagedata  = [
                            int(time()*1000000),    #time_usec
                            float(_temp[3]),               #xacc
                            float(_temp[4]),               #yacc
                            float(_temp[5]),               #zacc
                            float(_temp[6]),               #xgyro
                            float(_temp[7]),               #ygyro
                            float(_temp[8]),               #zgyro
                            float(_temp[11]),              #xmag
                            float(_temp[12]),              #ymag
                            float(_temp[13]),              #zmag
                            0,                      #abs_pressure
                            0,                      #diff_pressure
                            float(_temp[9]),               #pressure_alt
                            0,                      #temperature
                            0                       #fields_updated
                       ]

        return _messagedata
