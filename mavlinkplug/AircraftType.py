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

class AircraftTemplate(multiprocessing.Process):
    _data_out = [
        # for HIL_GPS
        'position/lat-gc-rad',          #0
        'position/long-gc-rad',         #1
        'position/h-sl-ft',             #2
        # for HIL_SENSOR
        'accelerations/udot-ft_sec2',   #3
        'accelerations/vdot-ft_sec2',   #4
        'accelerations/wdot-ft_sec2',   #5
        'velocities/p-aero-rad_sec',    #6
        'velocities/q-aero-rad_sec',    #7
        'velocities/r-aero-rad_sec',    #8
        'atmosphere/P-psf',             #9
        'atmosphere/pressure-altitude', #10
        'attitude/heading-true-rad',    #11
        'sensors/magnetometer/X/output',#12
        'sensors/magnetometer/Y/output',#13
        'sensors/magnetometer/Z/output',#14
        # for HIL_STATE
        'velocities/v-north-fps',       #15
        'velocities/v-east-fps',        #16
        'velocities/v-down-fps',        #17
        'velocities/vtrue-fps',         #18
        'velocities/vc-fps',            #19
        'attitude/phi-rad',             #20
        'attitude/theta-rad',           #21
        'attitude/psi-rad'              #22
    ]

class Plane(AircraftTemplate):

    _data_in = [
    'fcs/aileron-cmd-norm',
    'fcs/elevator-cmd-norm',
    'fcs/rudder-cmd-norm',
    'fcs/throttle-cmd-norm'
    ]

    JSBSIM_DEFAULT_PATH = path.dirname(__file__) + sep + 'data' + sep
    MIN_SERVO_PPM = 950
    MAX_SERVO_PPM = 1800

    def __init__(self, zmq_context = None, zmq_in = None, zmq_out = None, daemon = True, lat = 0, long = 0, terrain_elev_ft = 0, h_agl_ft = 1000, fdm_model = 'EasyStar', jsbsim_root = None, dt = 1.0/30):
        super(Plane,self).__init__()
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
        self._fdm.set_property_value("ic/theta-deg",0.0)
        self._fdm.set_property_value("ic/phi-deg",0.0)
        self._fdm.set_property_value("ic/psi-true-deg",110.0)
        self._fdm.set_property_value("ic/vt-kts",20)
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
    def cmd_norm(cls, value):
        """
        Normalize servo output data
        :param value: raw value of servo output
        :return:    normalized value of servo output
        """
        return (value - cls.MIN_SERVO_PPM)/(cls.MAX_SERVO_PPM - cls.MIN_SERVO_PPM)
    @classmethod
    def mav_2_FL(cls, mavlink_msg):
        """
        Adapting parameters from SERVO_OUTPUT_RAW type MAVlink message to FL (JSBsim)
        Class method for multiprocess purpose
        :param mavlink_msg: mavlink message
        :return: tuples including servo raw values or None if message not
        """
        if(mavlink_msg.get_type() == 'SERVO_OUTPUT_RAW'):
            return (
                    cls.cmd_norm(float(mavlink_msg.__dict__['servo1_raw'])),
                    cls.cmd_norm(float(mavlink_msg.__dict__['servo2_raw'])),
                    cls.cmd_norm(float(mavlink_msg.__dict__['servo3_raw'])),
                    cls.cmd_norm(float(mavlink_msg.__dict__['servo4_raw']))
                    )
        return None
    @classmethod
    def FL_2_mav_sensor(cls, string):
        """
        :param string: ZMQ message string including  _data_out values (Human Readable)
        :return: function parameter for mavlink message "MAVLink_hil_sensor_message"
        """
        temp = string.split(" ")

        #Data treatment
        messagedata  = [
                            int(time()*1000000),           #time_usec
                            float(temp[3]),                #xacc
                            float(temp[4]),                #yacc
                            float(temp[5]),                #zacc
                            float(temp[6]),                #xgyro
                            float(temp[7]),                #ygyro
                            float(temp[8]),                #zgyro
                            float(temp[12]),               #xmag
                            float(temp[13]),               #ymag
                            float(temp[14]),               #zmag
                            float(temp[9])*0.478802589,    #abs_pressure in millibar
                            0,                             #diff_pressure
                            float(temp[10]),               #pressure_alt
                            15.0,                          #temperature
                            int(65535)                     #fields_updated (ALL)
                       ]

        return messagedata
    @classmethod
    def FL_2_mav_state(cls,string):
        """
        :param string: ZMQ message string including  _data_out values (Human Readable)
        :return: function parameter for mavlink message "MAVLink_hil_state_quaternion_message"
        """
        temp = string.split(" ")

        #Data treatment
        messagedata  = []

        return messagedata