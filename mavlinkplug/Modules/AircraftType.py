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
from math import cos, sin, pi, sqrt

class AircraftTemplate(multiprocessing.Process):
    _g = 9.80665
    _sec2usec = 1.0e6
    _sec2msec = 1.0e3
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
    def FL_2_mav_sensor(cls, msg):
        """
        Translate output "data_out" to parameter for MAVLink_hil_sensor_message Pymavlink function
        :param string: ZMQ message string including  _data_out values (Human Readable)
        :return: function parameter for mavlink message "MAVLink_hil_sensor_message"
        """
        data = cls.message2data(msg)

        (data['velocities/phidot-rad_sec_bf'],
         data['velocities/thetadot-rad_sec_bf'],
         data['velocities/psidot-rad_sec_bf']) = cls.convert_body_frame(
                data['attitude/phi-rad'],
                data['attitude/theta-rad'],
                data['velocities/phidot-rad_sec'],
                data['velocities/thetadot-rad_sec'],
                data['velocities/psidot-rad_sec']
        )

        #Data treatment
        messagedata  = [
                            int(data['simulation/sim-time-sec']*cls._sec2usec),     #time_usec  boot time usec  int
                            data['accelerations/udot-ft_sec2'],                     #xacc   m/s**2  float
                            data['accelerations/vdot-ft_sec2'],                     #yacc   m/s**2  float
                            data['accelerations/wdot-ft_sec2'],                     #zacc   m/s**2  float
                            data['velocities/phidot-rad_sec_bf'],                   #xgyro  rad/s   float
                            data['velocities/thetadot-rad_sec_bf'],                 #ygyro  rad/s   float
                            data['velocities/psidot-rad_sec_bf'],                   #zgyro  rad/s   float
                            data['sensors/magnetometer/X/output'],                  #xmag   Gauss   float
                            data['sensors/magnetometer/Y/output'],                  #ymag   Gauss   float
                            data['sensors/magnetometer/Z/output'],                  #zmag   Gauss   float
                            data['atmosphere/P-psf']*0.478802589,                   #abs_pr mbar    float
                            0.0,                                                    #dif_pr mbar    float
                            data['atmosphere/pressure-altitude'],                   #pr_alt meter   float
                            15.0,                                                   #temp   C       float
                            int(8186)                                              #fields_updated (ALL)
                       ]

        return messagedata

    @classmethod
    def FL_2_mav_gps(cls, msg):
        """
        Translate output "data_out" to parameter for MAVLink_hil_gps_message Pymavlink function
        :param string: ZMQ message string including  _data_out values (Human Readable)
        :return: function parameter for mavlink message "MAVLink_hil_gps_message"
        """
        data = cls.message2data(msg)


        #Data treatment
        messagedata  = [
                            int(data['simulation/sim-time-sec']*cls._sec2usec),     #time_usec  boot time usec  int
                            3,                                                      #Fix_type    uint8_t	0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
                            int(data['position/lat-gc-rad']*cls._rad2degE7),        # lat            10e7.deg    int
                            int(data['position/long-gc-rad']*cls._rad2degE7),       # lon            10e7.deg    int
                            int(data['position/h-sl-ft']*cls._ft2m*cls._m2mm),      # alt            mm          int
                            65535,                                                  #eph    uint16_t	GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
                            65535,                                                  #epv	uint16_t	GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
                            65535,                                                  #uint16_t	GPS ground speed (m/s * 100). If unknown, set to: 65535
                            int(data['velocities/v-north-fps']*cls._ft2m*cls._m2cm),# vn             cm.s-1      int
                            int(data['velocities/v-east-fps']*cls._ft2m*cls._m2cm), # ve             cm.s-1      int
                            int(data['velocities/v-down-fps']*cls._ft2m*cls._m2cm), # vd             cm.s-1      int
                            65535,                                                  #cog	uint16_t	Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
                            255,                                                    #satellites_visible	uint8_t	Number of satellites visible. If unknown, set to 255
                       ]

        return messagedata

    @classmethod
    def FL_2_mav_state_quaternion(cls, msg):
        """
        Translate output "data_out" to parameter for MAVLink_hil_state_quaternion_message Pymavlink function
        :param string: ZMQ message string including  _data_out values (Human Readable)
        :return: function parameter for mavlink message "MAVLink_hil_state_quaternion_message"
        """
        data = cls.message2data(msg)

        #Attitude quaternion
        quaternion = cls.attitude_quaternion(cls, data['attitude/phi-rad'], data['attitude/theta-rad'], data['attitude/psi-rad'])

        #Data treatment
        messagedata  = [
                        int(data['simulation/sim-time-sec']*cls._sec2msec), #time_msec
                        quaternion,                                         #attitude_quaternion
                        data['velocities/phidot-rad_sec'],                  #rollspeed
                        data['velocities/thetadot-rad_sec'],                #pitchspeed
                        data['velocities/psidot-rad_sec'],                  #yawspeed
                        data['position/lat-gc-rad'],                        #lat
                        data['position/long-gc-rad'],                       #lon
                        data['position/h-sl-ft'],                           #alt
                        data['velocities/v-north-fps'],                     #vx
                        data['velocities/v-east-fps'],                      #vy
                        data['velocities/v-down-fps'],                      #vz
                        data['velocities/vc-fps'],                          #ind airspeed
                        data['velocities/vtrue-fps'],                       #true airspeed
                        data['accelerations/udot-ft_sec2'],                 #xacc
                        data['accelerations/vdot-ft_sec2'],                 #yacc
                        data['accelerations/wdot-ft_sec2'],                 #zacc
                        ]
        return messagedata

    @classmethod
    def FL_2_mav_state(cls, msg):
        """
        Translate output "data_out" to parameter for MAVLink_hil_state_message Pymavlink function
        :param msg: ZMQ message string including  _data_out values (Human Readable)
        :return: function parameter for mavlink message "MAVLink_hil_state_quaternion_message"
        """

        data = cls.message2data(msg)

        (data['velocities/phidot-rad_sec_bf'],
         data['velocities/thetadot-rad_sec_bf'],
         data['velocities/psidot-rad_sec_bf']) = cls.convert_body_frame(
                data['attitude/phi-rad'],
                data['attitude/theta-rad'],
                data['velocities/phidot-rad_sec'],
                data['velocities/thetadot-rad_sec'],
                data['velocities/psidot-rad_sec']
        )



        #Data treatment
        message_data = [
            int(data['simulation/sim-time-sec']*cls._sec2usec),                 # time           usec
            data['attitude/phi-rad'],                                           # phi            rad         float
            data['attitude/theta-rad'],                                         # theta          rad         float
            data['attitude/psi-rad'],                                           # psi            rad         float
            data['velocities/phidot-rad_sec_bf'],                               # rollspeed      rad.s-1     float
            data['velocities/thetadot-rad_sec_bf'],                             # pitchspeed     rad.s-1     float
            data['velocities/psidot-rad_sec_bf'],                               # yawspeed       rad.s-1     float
            int(data['position/lat-gc-rad']*cls._rad2degE7),                    # lat            10e7.deg    int
            int(data['position/long-gc-rad']*cls._rad2degE7),                   # lon            10e7.deg    int
            int(data['position/h-sl-ft']*cls._ft2m*cls._m2mm),                  # alt            mm          int
            int(data['velocities/v-north-fps']*cls._ft2m*cls._m2cm),            # vx             cm.s-1      int
            int(data['velocities/v-east-fps']*cls._ft2m*cls._m2cm),             # vy             cm.s-1      int
            int(data['velocities/v-down-fps']*cls._ft2m*cls._m2cm),             # vz             cm.s-1      int
            int(data['accelerations/udot-ft_sec2']*cls._ft2m*cls._mpss2mg),     # xacc           1000/g      int
            int(data['accelerations/vdot-ft_sec2']*cls._ft2m*cls._mpss2mg),     # yacc           1000/g      int
            int(data['accelerations/wdot-ft_sec2']*cls._ft2m*cls._mpss2mg)      # zacc           1000/g      int
        ]
        return message_data

    @classmethod
    def convert_body_frame(cls, phi, theta, phiDot, thetaDot, psiDot):
        '''convert a set of roll rates from earth frame to body frame'''
        p = phiDot - psiDot*sin(theta)
        q = cos(phi)*thetaDot + sin(phi)*psiDot*cos(theta)
        r = cos(phi)*psiDot*cos(theta) - sin(phi)*thetaDot
        return (p, q, r)

    @classmethod
    def message2data(cls,msg):
        temp = [float(i) for i in msg.split(" ")]
        return dict(zip(cls._data_out,temp))
    @classmethod
    def kinetic_energy(cls, msg):
        data = cls.message2data(msg)
        return   0.5*sqrt((data['velocities/v-north-fps']*cls._ft2m)**2
                      + (data['velocities/v-east-fps']*cls._ft2m)**2
                      + (data['velocities/v-down-fps']*cls._ft2m)**2)
    @classmethod
    def potential_energy(cls,msg):
        data = cls.message2data(msg)
        return data['position/h-sl-ft']*cls._ft2m*cls._g


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

    JSBSIM_DEFAULT_PATH = path.dirname(__file__) + sep + '..' + sep + 'data' + sep
    MIN_SERVO_PPM = 990
    MAX_SERVO_PPM = 2010
    THR_MIN = 0.0
    THR_MAX = 1.0
    CMD_MIN = -1.0
    CMD_MAX = 1.0

    def __init__(self, zmq_context = None, zmq_in = None, zmq_out = None, daemon = True, lat=43.6042600, lon=1.4436700, terrain_elev_ft = 400, h_agl_ft = 10000, fdm_model = 'easystar', jsbsim_root = None, dt = 1.0/30):
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
        self._fdm.set_property_value("ic/vt-kts",20)
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
