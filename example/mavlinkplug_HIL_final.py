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
    
    #Core module
    from time import sleep
    import logging
    
    #Internal module
    import mavlinkplug.Module
    import mavlinkplug.Plug
    #import mavlinkplug.QuadCopter
    import mavlinkplug.Hil
    
    if(_LOG == True):
        logger = logging.getLogger()
        logger.setLevel(logging.DEBUG)
        
        c_handler = logging.StreamHandler()
        f_handler = logging.FileHandler('mavlink.log',mode = 'w')
        c_handler.setLevel(logging.INFO)
        f_handler.setLevel(logging.DEBUG)
        
        formatter = logging.Formatter('%(asctime)s - %(processName)s - %(threadName)s - %(levelname)s - %(message)s')
        c_handler.setFormatter(formatter)
        f_handler.setFormatter(formatter)
        
        logger.addHandler(c_handler)
        logger.addHandler(f_handler)
    
    #Creating plug
    plug = mavlinkplug.Plug.Plug()
    
    #Creating Mavlink Connection 
    mavlink_con = mavlinkplug.Module.MAVlinkPlugConnection(plug.plug_info(),'2014-12-10 15-45-32.tlog')
    
    #Creating HIL environment
    #hil_env = mavlinkplug.Hil.MAVLinkPlugHil(plug.plug_info(), mavlink_con.ident(), mavlinkplug.QuadCopter.QuadCopter)
    
    plug.start()
    mavlink_con.start()
    #hil_env.run()
    #hil_env.FL_initialize()
    #hil_env.hardware_initialize()
    
    plug.server_forever()
    #sleep(5)
    
   # hil_env.stop()
    mavlink_con.stop()
    del(plug)