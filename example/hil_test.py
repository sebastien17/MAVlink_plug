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
import mavlinkplug.hil
from pyfdm import fdmexec
from pyfdm.exchange import zmq_exchange

#Create Plug
my_plug = mavlinkplug.Plug()

#Set a output file
my_plug.FILE_out('hil_test.txt')

#Connect to UAV on COM 3
mav_con01 = my_plug.MAVLINK_in('COM3',dialect='pixhawk')


#Flight Loop setup
JSBSIM_ROOT = mavlinkplug.hil.DATA_PATH
fdm = fdmexec.FGFDMExec(root_dir=JSBSIM_ROOT)
fdm.load_model("FG_c172p")
#Initial Conditions
fdm.set_property_value("ic/lat-gc-rad", 0.76130117)
fdm.set_property_value("ic/long-gc-rad",0.0239400705)
fdm.set_property_value("ic/terrain-elevation-ft", 150)
fdm.set_property_value("ic/h-agl-ft",2)
#Trim
fdm.do_trim(1)
#Defining exchange parameters
_IN = [ 'fcs/throttle-cmd-norm[0]',
        'fcs/throttle-cmd-norm[1]',
        'fcs/throttle-cmd-norm[2]',
        'fcs/throttle-cmd-norm[3]'
        ]

_OUT = [
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

#Defining zmq exchange parameters
zmq_tool = zmq_exchange(_IN,_OUT ,'tcp://127.0.0.1:17171','tcp://localhost:17172' )
fdm.exchange_register(zmq_tool)
fdm.realtime(dt=1.0/100)


#Set a HIL 
hil_01 = my_plug.hil(mav_con01, 'to_FL_zmq_sock_port', 'from_FL_zmq_sock_port','FL_instance')

hil_01.hardware_initialize()

my_plug.server_forever()