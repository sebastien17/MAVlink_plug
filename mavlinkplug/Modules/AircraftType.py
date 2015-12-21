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
import multiprocessing
from os import path, sep
from pyfdm import fdmexec
from pyfdm.exchange import zmq_exchange
from zmq import Context
from time import time
from math import cos, sin, pi

class AircraftTemplate(multiprocessing.Process):
    _g = 9.80665
    _sec2usec = 1.0e6
    _rad2degE7 = 1.0e7*180.0/pi
    _deg2rad = pi/180.0
    _m2mm = 1000.0
    _m2cm = 100.0
    _ft2m = 0.3048
    _mpss2mg =1000.0/_g

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

    @classmethod
    def attitude_quaternion(cls, phi, theta, psi):
        return [
            cos(phi/2)*cos(theta/2)*cos(psi/2)+sin(phi/2)*sin(theta/2)*sin(psi/2),
            sin(phi/2)*cos(theta/2)*cos(psi/2)-cos(phi/2)*sin(theta/2)*sin(psi/2),
            cos(phi/2)*sin(theta/2)*cos(psi/2)+sin(phi/2)*cos(theta/2)*sin(psi/2),
            cos(phi/2)*cos(theta/2)*sin(psi/2)-sin(phi/2)*sin(theta/2)*cos(psi/2)
            ]
    @classmethod
    def FL_2_mav_sensor(cls, string):
        """
        Translate output "data_out" to parameter for MAVLink_hil_sensor_message Pymavlink function
        :param string: ZMQ message string including  _data_out values (Human Readable)
        :return: function parameter for mavlink message "MAVLink_hil_sensor_message"
        """
        temp = string.split(" ")

        #Data treatment
        messagedata  = [
                        int(time()*1000),                                       #time_msec
                            float(temp[3]),                                     #xacc
                            float(temp[4]),                                     #yacc
                            float(temp[5]),                                     #zacc
                            float(temp[6]),                                     #xgyro
                            float(temp[7]),                                     #ygyro
                            float(temp[8]),                                     #zgyro
                            float(temp[12]),                                    #xmag
                            float(temp[13]),                                    #ymag
                            float(temp[14]),                                    #zmag
                            float(temp[9])*0.478802589,                         #abs_pressure in millibar
                            0,                                                  #diff_pressure
                            float(temp[10]),                                    #pressure_alt
                            15.0,                                               #temperature
                            int(65535)                                          #fields_updated (ALL)
                       ]

        return messagedata
    @classmethod
    def FL_2_mav_state_quaternion(cls, string):
        """
        Translate output "data_out" to parameter for MAVLink_hil_state_quaternion_message Pymavlink function
        :param string: ZMQ message string including  _data_out values (Human Readable)
        :return: function parameter for mavlink message "MAVLink_hil_state_quaternion_message"
        """
        temp = string.split(" ")

        #Data treatment
        messagedata  = [
                        int(time()*1000),                                       #time_msec
                        cls.attitude_quaternion(cls, float(temp[20]), float(temp[21]), float(temp[22])),    #attitude_quaternion
                        float(temp[6]),                                         #rollspeed
                        float(temp[7]),                                         #pitchspeed
                        float(temp[8]),                                         #yawspeed
                        float(temp[0]),                                         #lat
                        float(temp[1]),                                         #lon
                        float(temp[2]),                                         #alt
                        float(temp[15]),                                        #vx
                        float(temp[16]),                                        #vy
                        float(temp[17]),                                        #vz
                        float(temp[19]),                                        #ind airspeed
                        float(temp[18]),                                        #true airspeed
                        float(temp[3]),                                         #xacc
                        float(temp[4]),                                         #yacc
                        float(temp[5]),                                         #zacc
        ]
        return messagedata
    @classmethod
    def FL_2_mav_state(cls, string):
        """
        Translate output "data_out" to parameter for MAVLink_hil_state_message Pymavlink function
        :param string: ZMQ message string including  _data_out values (Human Readable)
        :return: function parameter for mavlink message "MAVLink_hil_state_quaternion_message"
        """
        temp = [float(i) for i in string.split(" ")]

        #Data treatment
        messagedata  = [
                        int(temp[23]*cls._sec2usec),                            # time           usec
                        temp[20],                                               # phi            rad         float
                        temp[21],                                               # theta          rad         float
                        temp[22],                                               # psi            rad         float
                        temp[6],                                                # rollspeed      rad.s-1     float
                        temp[7],                                                # pitchspeed     rad.s-1     float
                        temp[8],                                                # yawspeed       rad.s-1     float
                        int(temp[0]*cls._rad2degE7),                            # lat            10e7.deg    int
                        int(temp[1]*cls._rad2degE7),                            # lon            10e7.deg    int
                        int(temp[2]*cls._ft2m*cls._m2mm),                       # alt            mm          int
                        int(temp[15]*cls._ft2m*cls._m2cm),                      # vx             cm.s-1      int
                        int(temp[16]*cls._ft2m*cls._m2cm),                      # vy             cm.s-1      int
                        int(temp[17]*cls._ft2m*cls._m2cm),                      # vz             cm.s-1      int
                        int(temp[3]*cls._ft2m*cls._mpss2mg),                    # xacc           1000/g      int
                        int(temp[4]*cls._ft2m*cls._mpss2mg),                    # yacc           1000/9      int
                        int(temp[5]*cls._ft2m*cls._mpss2mg),                    # zacc           1000/g      int
        ]
        print(messagedata)
        return messagedata

class Plane(AircraftTemplate):

    _data_in = [
    'fcs/aileron-cmd-norm',
    'fcs/elevator-cmd-norm',
    'fcs/rudder-cmd-norm',
    'fcs/throttle-cmd-norm',
    'atmosphere/wind-north-fps',
    'atmosphere/wind-east-fps',
    'atmosphere/wind-down-fps'
    ]

    JSBSIM_DEFAULT_PATH = path.dirname(__file__) + sep + 'data' + sep
    MIN_SERVO_PPM = 990
    MAX_SERVO_PPM = 2010
    THR_MIN = 0.0
    THR_MAX = 1.0
    CMD_MIN = -1.0
    CMD_MAX = 1.0

    def __init__(self, zmq_context = None, zmq_in = None, zmq_out = None, daemon = True, lat=43.6042600, lon=1.4436700, terrain_elev_ft = 400, h_agl_ft = 10000, fdm_model = 'c172p', jsbsim_root = None, dt = 1.0/30):
        super(Plane,self).__init__()
        self._zmq_context = zmq_context
        self._zmq_in = zmq_in
        self._zmq_out = zmq_out
        self.daemon = daemon
        self._lat_rad = lat*self._deg2rad
        self._long_rad = lon*self._deg2rad
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
        self._fdm.set_property_value("ic/phi-deg",0.0)                                      # Roll
        # self._fdm.set_property_value("ic/theta-deg",0.0)                                    # Pitch
        self._fdm.set_property_value("ic/psi-true-deg",110.0)                               # Heading
        self._fdm.set_property_value("ic/vt-kts",90)
        #Fdm Trim
        #self._fdm.do_trim(1)
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
        Normalize servo output data for command
        :param value: raw value of servo output
        :return:    normalized value of servo output for command
        """
        value = float(value)
        value01 = (value - cls.MIN_SERVO_PPM)/(cls.MAX_SERVO_PPM - cls.MIN_SERVO_PPM)
        return (cls.CMD_MAX - cls.CMD_MIN)*value01 + cls.CMD_MIN
    @classmethod
    def thr_norm(cls, value):
        """
        Normalize servo output data for throttle
        :param value: raw value of servo output
        :return:    normalized value of servo output for throttle
        """
        value = float(value)
        value01 = (value - cls.MIN_SERVO_PPM)/(cls.MAX_SERVO_PPM - cls.MIN_SERVO_PPM)
        return (cls.THR_MAX - cls.THR_MIN)*value01 + cls.THR_MIN
    @classmethod
    def mav_2_FL(cls, mavlink_msg, wind_data):
        """
        Adapt parameters from SERVO_OUTPUT_RAW type MAVlink message to FL (JSBsim)
        Class method for multiprocess purpose
        :param mavlink_msg: mavlink message
        :return: tuples including servo raw values or None if message not
        """
        if(mavlink_msg.get_type() == 'SERVO_OUTPUT_RAW'):
            return (
                    -cls.cmd_norm(mavlink_msg.__dict__['servo1_raw']),
                    cls.cmd_norm(mavlink_msg.__dict__['servo2_raw']),
                    cls.thr_norm(mavlink_msg.__dict__['servo3_raw']),
                    cls.cmd_norm(mavlink_msg.__dict__['servo4_raw']),
                    wind_data[0],
                    wind_data[1],
                    wind_data[2],
                    )
        return None
