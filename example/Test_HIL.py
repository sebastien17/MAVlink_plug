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

    from time import sleep
    import logging
    
    #Internal module
    import mavlinkplug.Module
    import mavlinkplug.Plug
    import mavlinkplug.AircraftType
    import mavlinkplug.Hil
    

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
    file_output = mavlinkplug.Module.MAVlinkPlugFileWriter(plug.plug_info(),'Test_HIL.log')
    file_output.start()

    #Set a mavlink connection with  MAVlink ready devices
    mav_con_01 = mavlinkplug.Module.MAVlinkPlugConnection(plug.plug_info(),'COM3',dialect='ardupilotmega',baud=115200)
    mav_con_01.start() #Mavlink connection start

    #Set a connection for GC
    gc_connection = mavlinkplug.Module.MAVLinkPlugTCPConnection(plug.plug_info(), ('',17562), mav_con_01.ident())
    gc_connection.start()

    #Creating HIL environment
    hil_env = mavlinkplug.Hil.MAVLinkPlugHil(plug.plug_info(), mav_con_01.ident(), mavlinkplug.AircraftType.Plane)


    #Automatic Launch Procedure
    hil_env.start()

    #Server forever
    plug.server_forever() #Wait forever

    
