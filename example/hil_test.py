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
#Setting JSBSIM_ROOT path
JSBSIM_ROOT = os.path.abspath('data/jsbsim_data') + os.sep

#Initializing FGFDMExec class
fdm = fdmexec.FGFDMExec(root_dir=JSBSIM_ROOT)

#Loading aircraft model
fdm.load_model("FG_c172p")

#Initializing Aircraft
fdm.set_property_value("fcs/mixture-cmd-norm",1.0)
fdm.set_property_value("propulsion/magneto_cmd",3)
fdm.set_property_value("propulsion/starter_cmd",1)
fdm.set_property_value("ic/lat-gc-rad",0.761552988)
fdm.set_property_value("ic/long-gc-rad",0.0239284344)
fdm.set_property_value("ic/h-agl-ft",1000)
fdm.set_property_value("ic/vc-kts",80)
fdm.set_property_value("ic/gamma-deg",0)
#Trim
fdm.do_trim(1)


#Defining parameters
_IN = ['fcs/aileron-cmd-norm',
'fcs/elevator-cmd-norm',
'fcs/rudder-cmd-norm',
'fcs/flap-cmd-norm',
'fcs/speedbrake-cmd-norm',
'fcs/spoiler-cmd-norm',
'fcs/pitch-trim-cmd-norm',
'fcs/roll-trim-cmd-norm',
'fcs/yaw-trim-cmd-norm',
'fcs/left-brake-cmd-norm',
'fcs/right-brake-cmd-norm',
'fcs/steer-cmd-norm',
'fcs/throttle-cmd-norm',
'fcs/mixture-cmd-norm']

_OUT = ['position/h-sl-ft',
'position/h-sl-meters',
'position/lat-gc-rad',
'position/long-gc-rad',
'position/lat-gc-deg',
'position/long-gc-deg',
'position/h-agl-ft',
'position/h-agl-km',
'position/terrain-elevation-asl-ft']

#Defining zmq exchange parameters
zmq_tool = zmq_exchange(_IN,_OUT ,'tcp://127.0.0.1:17171','tcp://localhost:17172' )



#Registering exchange class to FGFDMExec class
fdm.exchange_register(zmq_tool)



#List of exchange class registered
fdm.list_exchange_class()

#Running FDM loop
fdm.realtime(dt=1.0/100)


#Set a HIL 
hil_01 = my_plug.hil(mav_con01, 'to_FL_zmq_sock_port', 'from_FL_zmq_sock_port','FL_instance')

hil_01.hardware_initialize()

my_plug.server_forever()