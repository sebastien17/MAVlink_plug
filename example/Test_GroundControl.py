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

if(__name__ == '__main__'):

    import mavlinkplug.Modules.MavConnection
    import mavlinkplug.Modules.FileWriter
    import mavlinkplug.Plug
    import logging

    #Handling logging options
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    handler = logging.StreamHandler()
    handler.setLevel(logging.INFO)
    formatter = logging.Formatter(' %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)

    #Creating plug
    plug = mavlinkplug.Plug.Plug()
    plug.start()

    #Set a mavlink connection with  MAVlink ready devices
    mav_con_01 = mavlinkplug.Modules.MavConnection.MavConnection(plug.plug_info(), 'COM7', dialect='ardupilotmega', baud=115200)

    #Set a output file
    file_output = mavlinkplug.Modules.FileWriter.FileWriter(plug.plug_info(), 'Test_GroundControl.log')

    #Start all modules
    file_output.start()
    gc_connection.start()
    mav_con_01.start()

    #Server forever
    plug.server_forever()