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
import mavlinkplug.Message, mavlinkplug.Tools
from tornado import gen, ioloop

class Hil(ZmqBase):

    _g = 9.80665
    _sec2usec = 1.0e6
    _sec2msec = 1.0e3
    _rad2deg = 180.0/math.pi
    _rad2degE7 = 1.0e7*180.0/math.pi
    _deg2rad = math.pi/180.0
    _m2mm = 1000.0
    _m2cm = 100.0
    _ft2m = 0.3048
    _mpss2mg =1000.0/_g
    ENERGY_MSG_HEADER  = 'KE_PE'

    _data_FL_out = [
    'simulation/sim-time-sec',
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
    'attitude/psi-rad'              #22
    ]

    def __init__(self, module_info, mavlink_connection_ident, Aircraft_Type_cls, hil_sensor=True, quaternion=True, state_freq = 30.0, sensor_freq = 20.0, gps_freq = 5.0,  name=None):
        super(Hil, self).__init__()
        self._mavlink_connection_ident = mavlink_connection_ident
        self._addr_to_plug, self._addr_from_plug, self._ident =  module_info
        self._addr_to_FL = 'tcp://127.0.0.1:45063'
        self._addr_from_FL = 'tcp://127.0.0.1:45064'
        self._Aircraft_Type_cls = Aircraft_Type_cls
        self._default_subscribe.append(mavlinkplug.Message.integer_pack(self._ident))
        self._dumb_header = mavlinkplug.Message.mavlink.MAVLink_header(0)
        self._aircraft = None
        self._phase = 0
        self._hb_count = 0
        self._hil_sensor = hil_sensor
        self._quaternion = quaternion
        self._thermal_wind_NED = (0.0, 0.0, 0.0)
        self._thermal_list = []
        self._last_FL_send_time = self._last_energy_send_time = self._last_state_send_time = self._last_sensor_send_time = self._last_gps_send_time = time()
        self._state_period = 1.0/state_freq
        self._sensor_period = 1.0/sensor_freq
        self._gps_period = 1.0/gps_freq
        self._last_jsbsim_msg = None
        self._last_mav_msg4FL = None
        self._last_mav_msg4FL_time = time()

        if(name == None ):
            self._name = 'Hil_' + str(self._ident)
        else:
            self._name = name

    def setup(self):
        super(Hil, self).setup()
        # Initializing message callback
        # Define stream listening from plug
        self.stream(zmq.SUB, self._addr_from_plug, bind = False, callback = self._plug_2_FL)
        #Define stream publishing to plug
        self._stream2Plug  = self.stream(zmq.PUB, self._addr_to_plug, bind = False)
        #Install scheduler in IOloop
        self._loop.add_callback(self._scheduler)

        #Define stream listening from FL
        self._loop.add_handler(self._aircraft.socket_out().fileno(), self._FL_2_plug, ioloop.IOLoop.READ)

    @gen.engine
    def _scheduler(self):
        while self._running:
            if self._last_jsbsim_msg != None:
                actual_time = time()
                #State send
                if(actual_time - self._last_state_send_time > self._state_period):
                    self._last_state_send_time = actual_time
                    self._send_state_frame()
                #Sensor send
                if(actual_time - self._last_sensor_send_time > self._sensor_period and self._hil_sensor == True):
                    self._last_sensor_send_time = actual_time
                    self._send_sensor_frame()
                #Gps send
                if(actual_time - self._last_gps_send_time > self._gps_period and self._hil_sensor == True):
                    self._last_gps_send_time = actual_time
                    self._send_gps_frame()
                #Energy send
                if(actual_time - self._last_energy_send_time > 1.0/5):
                    self._last_energy_send_time = actual_time
                    self._send_energy_frame()
                #FL send
                if(actual_time - self._last_FL_send_time > 1.0/5):
                    self._last_FL_send_time = actual_time
                    self._aircraft.send_prepared_msg()


            #60 Hz Cycle
            yield gen.Task(self._loop.current().add_timeout, time() + 1.0/60)

    def _get_MAVlink_Plug_Message(self, msg):
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
        self._aircraft = self._Aircraft_Type_cls()
        self._logging('INIT phase end')
        self._phase = 7 #Go to WAIT_FOR_ALIVE

    def _phase_WAIT_FOR_ALIVE(self,msg):
        if(self._hb_count == 0):
            self._logging('WAIT_FOR_ALIVE phase start')
        if(msg.header.type == mavlinkplug.Message.TYPE.MAV_MSG.value and msg.data.value.get_type() == 'HEARTBEAT'):
            self._hb_count += 1
        if(self._hb_count == 3):
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
            self._aircraft.run()
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
            self._aircraft.store_msg(msg.data.value, self._thermal_wind_NED)

    def _send_state_frame(self):
        try:
            if(self._quaternion):
                # HIL State Quaternion Mavlink Message Creation
                # 'time_usec', 'attitude_quaternion', 'rollspeed', 'pitchspeed', 'yawspeed',
                # 'lat', 'lon', 'alt', 'vx', 'vy', 'vz', 'ind_airspeed', 'true_airspeed', 'xacc', 'yacc', 'zacc']
                parameters = self.FL_2_mav_state_quaternion(self._last_jsbsim_msg)
                mav_message_state = mavlinkplug.Message.mavlink.MAVLink_hil_state_quaternion_message(*parameters).pack(self._dumb_header)
            else:
                # HIL State Mavlink Message Creation
                # 'time_usec', 'roll', 'pitch', 'yaw', 'rollspeed', 'pitchspeed', 'yawspeed',
                # 'lat', 'lon', 'alt', 'vx', 'vy', 'vz', 'xacc', 'yacc', 'zacc'
                parameters = self.FL_2_mav_state(self._last_jsbsim_msg)
                mav_message_state = mavlinkplug.Message.mavlink.MAVLink_hil_state_message(*parameters).pack(self._dumb_header)
        except Exception as e:
            self._logging(e.message)
        else:
            mavlink_plug_message = mavlinkplug.Message.MAVLinkData.build_full_message_from( self._mavlink_connection_ident, self._ident, long(time()), mav_message_state )
            self._stream2Plug.send(mavlink_plug_message.packed)

    def _send_sensor_frame(self):
        try:
            # HIL sensor Mavlink Message Creation
            # 'time_msec', 'xacc', 'yacc', 'zacc', 'xgyro', 'ygyro', 'zgyro', 'xmag', 'ymag', 'zmag',
            # 'abs_pressure in millibar', 'diff_pressure', 'pressure_alt', 'temperature', 'fields_updated'
            parameters_sensors = self.FL_2_mav_sensor(self._last_jsbsim_msg)
            mav_message_sensor = mavlinkplug.Message.mavlink.MAVLink_hil_sensor_message(*parameters_sensors).pack(self._dumb_header)

        except Exception as e:
            self._logging(e.message)
        else:
            mavlink_plug_message_sensor = mavlinkplug.Message.MAVLinkData.build_full_message_from( self._mavlink_connection_ident, self._ident,long(time()), mav_message_sensor )
            self._stream2Plug.send(mavlink_plug_message_sensor.packed)

    def _send_gps_frame(self):
        try:
            parameters_gps = self.FL_2_mav_gps(self._last_jsbsim_msg)
            mav_message_gps = mavlinkplug.Message.mavlink.MAVLink_hil_gps_message(*parameters_gps).pack(self._dumb_header)
        except Exception as e:
            self._logging(e.message)
        else:
            mavlink_plug_message_gps = mavlinkplug.Message.MAVLinkData.build_full_message_from( self._mavlink_connection_ident, self._ident,long(time()), mav_message_gps )
            self._stream2Plug.send(mavlink_plug_message_gps.packed)

    def _send_energy_frame(self):
        #Energy Information Message Creation
        raw_string = "{0} {1} {2}".format(self.ENERGY_MSG_HEADER, self.kinetic_energy(self._last_jsbsim_msg),self.potential_energy(self._last_jsbsim_msg))
        energy_message = mavlinkplug.Message.RawData.build_full_message_from(mavlinkplug.Message.DESTINATION.ALL.value, self._ident, long(time()), raw_string )
        self._stream2Plug.send(energy_message.packed)

    def _FL_2_plug(self, fd, events):

        msg = self._aircraft.socket_out().receive(1024)
        self._last_jsbsim_msg = self.message2data(msg)
        self._logging(self._last_jsbsim_msg)
        self._deg_coordinates_tuple = self.deg_coordinate_tuple(self._last_jsbsim_msg)
        self._thermals_management()


    def add_thermal(self, deg_lat_long_tuple, amplitude, D_param):
        self._thermal_list.append(tuple(deg_lat_long_tuple, amplitude, D_param))

    def _thermals_management(self):
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
         logging_message = mavlinkplug.Message.LogData.build_full_message_from( mavlinkplug.Message.DESTINATION.ALL.value,
                                                                                self._ident,
                                                                                long(time()),
                                                                                type+': '+ msg
                                                                                )
         self._stream2Plug.send(logging_message.packed)

#############################################################################


    def attitude_quaternion(self, phi, theta, psi):
        return [
            math.cos(phi/2)*math.cos(theta/2)*math.cos(psi/2)+math.sin(phi/2)*math.sin(theta/2)*math.sin(psi/2),
            math.sin(phi/2)*math.cos(theta/2)*math.cos(psi/2)-math.cos(phi/2)*math.sin(theta/2)*math.sin(psi/2),
            math.cos(phi/2)*math.sin(theta/2)*math.cos(psi/2)+math.sin(phi/2)*math.cos(theta/2)*math.sin(psi/2),
            math.cos(phi/2)*math.cos(theta/2)*math.sin(psi/2)-math.sin(phi/2)*math.sin(theta/2)*math.cos(psi/2)
               ]

    def FL_2_mav_sensor(self, data):
        """
        Translate output "data_out" to parameter for MAVLink_hil_sensor_message Pymavlink function
        :param string: ZMQ message string including  _data_out values (Human Readable)
        :return: function parameter for mavlink message "MAVLink_hil_sensor_message"
        """

        (data['velocities/phidot-rad_sec_bf'],
         data['velocities/thetadot-rad_sec_bf'],
         data['velocities/psidot-rad_sec_bf']) = self.convert_body_frame(
                data['attitude/phi-rad'],
                data['attitude/theta-rad'],
                data['velocities/phidot-rad_sec'],
                data['velocities/thetadot-rad_sec'],
                data['velocities/psidot-rad_sec']
        )

        #Data treatment
        messagedata  = [
                            int(data['simulation/sim-time-sec']*self._sec2usec),     #time_usec  boot time usec  int
                            data['accelerations/udot-ft_sec2'],                     #xacc   m/s**2  float
                            data['accelerations/vdot-ft_sec2'],                     #yacc   m/s**2  float
                            data['accelerations/wdot-ft_sec2'],                     #zacc   m/s**2  float
                            data['velocities/phidot-rad_sec'],                   #xgyro  rad/s   float
                            data['velocities/thetadot-rad_sec'],                 #ygyro  rad/s   float
                            data['velocities/psidot-rad_sec'],                   #zgyro  rad/s   float
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

    def FL_2_mav_gps(self, data):
        """
        Translate output "data_out" to parameter for MAVLink_hil_gps_message Pymavlink function
        :param string: ZMQ message string including  _data_out values (Human Readable)
        :return: function parameter for mavlink message "MAVLink_hil_gps_message"
        """



        #Data treatment
        messagedata  = [
                            int(data['simulation/sim-time-sec']*self._sec2usec),     #time_usec  boot time usec  int
                            3,                                                      #Fix_type    uint8_t	0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
                            int(data['position/lat-gc-rad']*self._rad2degE7),        # lat            10e7.deg    int
                            int(data['position/long-gc-rad']*self._rad2degE7),       # lon            10e7.deg    int
                            int(data['position/h-sl-ft']*self._ft2m*self._m2mm),      # alt            mm          int
                            65535,                                                  #eph    uint16_t	GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
                            65535,                                                  #epv	uint16_t	GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
                            65535,                                                  #uint16_t	GPS ground speed (m/s * 100). If unknown, set to: 65535
                            int(data['velocities/v-north-fps']*self._ft2m*self._m2cm),# vn             cm.s-1      int
                            int(data['velocities/v-east-fps']*self._ft2m*self._m2cm), # ve             cm.s-1      int
                            int(data['velocities/v-down-fps']*self._ft2m*self._m2cm), # vd             cm.s-1      int
                            65535,                                                  #cog	uint16_t	Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
                            255,                                                    #satellites_visible	uint8_t	Number of satellites visible. If unknown, set to 255
                       ]

        return messagedata

    def FL_2_mav_state_quaternion(self, data):
        """
        Translate output "data_out" to parameter for MAVLink_hil_state_quaternion_message Pymavlink function
        :param string: ZMQ message string including  _data_out values (Human Readable)
        :return: function parameter for mavlink message "MAVLink_hil_state_quaternion_message"
        """

        #Attitude quaternion
        quaternion = self.attitude_quaternion(data['attitude/phi-rad'], data['attitude/theta-rad'], data['attitude/psi-rad'])

        #Data treatment
        messagedata  = [
                        int(data['simulation/sim-time-sec']*self._sec2msec), #time_msec
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

    def FL_2_mav_state(self, data):
        """
        Translate output "data_out" to parameter for MAVLink_hil_state_message Pymavlink function
        :param msg: ZMQ message string including  _data_out values (Human Readable)
        :return: function parameter for mavlink message "MAVLink_hil_state_quaternion_message"
        """

        (data['velocities/phidot-rad_sec_bf'],
         data['velocities/thetadot-rad_sec_bf'],
         data['velocities/psidot-rad_sec_bf']) = self.convert_body_frame(
                data['attitude/phi-rad'],
                data['attitude/theta-rad'],
                data['velocities/phidot-rad_sec'],
                data['velocities/thetadot-rad_sec'],
                data['velocities/psidot-rad_sec']
        )

        # Data treatment
        message_data = [
            1000000000,
            # int(data['simulation/sim-time-sec']*cls._sec2usec),                 # time           usec
            data['attitude/phi-rad'],                                           # phi            rad         float
            data['attitude/theta-rad'],                                         # theta          rad         float
            data['attitude/psi-rad'],                                           # psi            rad         float
            data['velocities/phidot-rad_sec'],                               # rollspeed      rad.s-1     float
            data['velocities/thetadot-rad_sec'],                             # pitchspeed     rad.s-1     float
            data['velocities/psidot-rad_sec'],                               # yawspeed       rad.s-1     float
            int(data['position/lat-gc-rad']*self._rad2degE7),                    # lat            10e7.deg    int
            int(data['position/long-gc-rad']*self._rad2degE7),                   # lon            10e7.deg    int
            int(data['position/h-sl-ft']*self._ft2m*self._m2mm),                  # alt            mm          int
            int(data['velocities/v-north-fps']*self._ft2m*self._m2cm),            # vx             cm.s-1      int
            int(data['velocities/v-east-fps']*self._ft2m*self._m2cm),             # vy             cm.s-1      int
            int(data['velocities/v-down-fps']*self._ft2m*self._m2cm),             # vz             cm.s-1      int
            int(data['accelerations/udot-ft_sec2']*self._ft2m*self._mpss2mg),     # xacc           1000/g      int
            int(data['accelerations/vdot-ft_sec2']*self._ft2m*self._mpss2mg),     # yacc           1000/g      int
            int(data['accelerations/wdot-ft_sec2']*self._ft2m*self._mpss2mg)      # zacc           1000/g      int
        ]
        return message_data

    def convert_body_frame(self, phi, theta, phiDot, thetaDot, psiDot):
        '''convert a set of roll rates from earth frame to body frame'''
        p = phiDot - psiDot*math.sin(theta)
        q = math.cos(phi)*thetaDot + math.sin(phi)*psiDot*math.cos(theta)
        r = math.cos(phi)*psiDot*math.cos(theta) - math.sin(phi)*thetaDot
        return (p, q, r)

    def message2data(self,msg):
        temp = [float(i) for i in msg.split(" ")]
        return dict(zip(self._data_FL_out,temp))

    def deg_coordinate_tuple(self, data):

        return data['position/lat-gc-rad']*self._rad2deg, data['position/long-gc-rad']*self._rad2deg

    def kinetic_energy(self, data):
        return 0.5*math.sqrt((data['velocities/v-north-fps']*self._ft2m)**2
                      + (data['velocities/v-east-fps']*self._ft2m)**2
                      + (data['velocities/v-down-fps']*self._ft2m)**2)

    def potential_energy(self, data):
        return data['position/h-sl-ft']*self._ft2m*self._g
