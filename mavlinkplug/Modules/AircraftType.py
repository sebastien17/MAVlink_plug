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

# Core import
from __future__ import print_function

from os import path, sep
import math


class AircraftTemplate(object):
    
    _g = 9.80665
    _sec2usec = 1.0e6
    _sec2msec = 1.0e3
    _rad2deg = 180.0/math.pi
    _rad2degE7 = 1.0e7*180.0/math.pi
    _deg2rad = math.pi/180.0
    _m2mm = 1000.0
    _m2cm = 100.0
    _ft2m = 0.3048
    _mpss2mg = 1000.0/_g

    _data_out = [
        # for HIL_GPS
        'position/lat-gc-rad',          #0
        'position/long-gc-rad',         #1
        'position/h-sl-ft',             #2
        # for HIL_SENSOR
        'accelerations/udot-ft_sec2',   #3
        'accelerations/vdot-ft_sec2',   #4
        'accelerations/wdot-ft_sec2',   #5
        'velocities/phidot-rad_sec',    #6
        'velocities/thetadot-rad_sec',  #7
        'velocities/psidot-rad_sec',    #8
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
        'attitude/psi-rad',             #22
        'simulation/sim-time-sec'       #23
    ]


class EasyStar(object):

    MIN_SERVO_PPM = 990
    MAX_SERVO_PPM = 2010
    THR_MIN = 0.0
    THR_MAX = 1.0
    CMD_MIN = -1.0
    CMD_MAX = 1.0
    
    _data_in = [
        'fcs/aileron-cmd-norm',
        'fcs/elevator-cmd-norm',
        'fcs/rudder-cmd-norm',
        'fcs/throttle-cmd-norm',
        'atmosphere/wind-north-fps',
        'atmosphere/wind-east-fps',
        'atmosphere/wind-down-fps'
    ]



    def __init__(self):
        super(EasyStar,self).__init__()
        
        #Flight Loop setup
        #Initial Conditions
        self._fdm.set_property_value("ic/lat-gc-rad", self._lat_rad)
        self._fdm.set_property_value("ic/long-gc-rad",self._long_rad)
        self._fdm.set_property_value("ic/terrain-elevation-ft", self._terrain_elev_ft)
        self._fdm.set_property_value("ic/h-agl-ft",self._h_agl_ft)
        self._fdm.set_property_value("ic/phi-deg",0.0)                                      # Roll
        # self._fdm.set_property_value("ic/theta-deg",0.0)                                  # Pitch
        self._fdm.set_property_value("ic/psi-true-deg",110.0)                               # Heading
        self._fdm.set_property_value("ic/vt-kts",90)
        #Fdm Trim

    def run(self):
        self.setup()
        self._fdm.realtime(self._dt)

    def stop(self):
        self.terminate()

    def cmd_norm(self, value):
        """
        Normalize servo output data for command
        :param value: raw value of servo output
        :return:    normalized value of servo output for command
        """
        value = float(value)
        value01 = (value - self.MIN_SERVO_PPM)/(self.MAX_SERVO_PPM - self.MIN_SERVO_PPM)
        return (self.CMD_MAX - self.CMD_MIN)*value01 + self.CMD_MIN

    def thr_norm(self, value):
        """
        Normalize servo output data for throttle
        :param value: raw value of servo output
        :return:    normalized value of servo output for throttle
        """
        value = float(value)
        value01 = (value - self.MIN_SERVO_PPM)/(self.MAX_SERVO_PPM - self.MIN_SERVO_PPM)
        return (self.THR_MAX - self.THR_MIN)*value01 + self.THR_MIN

    def mav_2_FL(self, mavlink_msg, wind_data):
        """
        Adapt parameters from SERVO_OUTPUT_RAW type MAVlink message to FL (JSBsim)
        Class method for multiprocess purpose
        :param mavlink_msg: mavlink message
        :return: tuples including servo raw values or None if message not
        """
        if(mavlink_msg.get_type() == 'SERVO_OUTPUT_RAW'):
            return (
                    -self.cmd_norm(mavlink_msg.__dict__['servo1_raw']),              # 'fcs/aileron-cmd-norm'
                    self.cmd_norm(mavlink_msg.__dict__['servo2_raw']),               # 'fcs/elevator-cmd-norm'
                    self.thr_norm(mavlink_msg.__dict__['servo3_raw']),               # 'fcs/rudder-cmd-norm'
                    self.cmd_norm(mavlink_msg.__dict__['servo4_raw']),               # 'fcs/throttle-cmd-norm'
                    wind_data[0],                                                   # 'atmosphere/wind-north-fps'
                    wind_data[1],                                                   # 'atmosphere/wind-east-fps'
                    wind_data[2],                                                   # 'atmosphere/wind-down-fps'
                    )
        return None
