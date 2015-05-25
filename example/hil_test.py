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


#Set a HIL 
hil_01 = my_plug.hil(mav_con01, 'to_FL_zmq_sock_port', 'from_FL_zmq_sock_port','FL_instance')

hil_01.hardware_initialize()

my_plug.server_forever()