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
#External import
from pyfdm import fdmexec
from pyfdm.exchange import zmq_exchange
from zmq import Context


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
    def mav_2_FL(cls, mavlink_msg):
        def norm(value):
            return (value - cls.MIN_SERVO_PPM)/(cls.MAX_SERVO_PPM - cls.MIN_SERVO_PPM)
        if(mavlink_msg.get_type() == 'SERVO_OUTPUT_RAW'):
            return (
                    norm(float(mavlink_msg.__dict__['servo1_raw'])),
                    norm(float(mavlink_msg.__dict__['servo2_raw'])),
                    norm(float(mavlink_msg.__dict__['servo3_raw'])),
                    norm(float(mavlink_msg.__dict__['servo4_raw']))
                    )
        return None
    @classmethod
    def FL_2_mav(cls, string):
        _temp = []
        _messagedata = string.split(" ")
        #TODO : Data treatment
        _temp = _messagedata
        return _temp
        
        return _temp
