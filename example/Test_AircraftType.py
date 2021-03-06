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

_LOG = True

if(__name__ == '__main__'):

    import logging
    
    #Internal module
    import mavlinkplug.Modules.Module
    import mavlinkplug.Plug
    import mavlinkplug.Modules.AircraftType
    import mavlinkplug.Modules.Hil
    

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

    #Set a output file
    file_output = mavlinkplug.Modules.Module.MAVlinkPlugFileWriter(plug.plug_info(), 'Test_AircraftType.log')
    file_output.start()

    #Creating HIL environment
    hil_env = mavlinkplug.Modules.Hil.Hil(plug.plug_info(), 17, mavlinkplug.Modules.AircraftType.Plane)
    
    hil_env.FL_initialize() #Flight loop start
    hil_env.start() #HIL start

    #hil_env.hardware_initialize() #MAV start

    #Server forever
    plug.server_forever() #Wait forever

    
