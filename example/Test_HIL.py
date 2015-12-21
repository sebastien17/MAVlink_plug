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

if(__name__ == '__main__'):


    #Internal module
    import mavlinkplug.Plug
    import mavlinkplug.Modules.FileWriter
    import mavlinkplug.Modules.MavConnection
    import mavlinkplug.Modules.TcpConnection
    import mavlinkplug.Modules.Hil
    import mavlinkplug.Modules.AircraftType

        #Creating plug
    plug = mavlinkplug.Plug.Plug()
    plug.start()

    #Set a output file
    file_output = mavlinkplug.Modules.FileWriter.FileWriter(plug.plug_info(), 'Test_HIL.log')
    file_output.start()

    #Set a mavlink connection with  MAVlink ready devices
    mav_con_01 = mavlinkplug.Modules.MavConnection.MavConnection(plug.plug_info(), 'COM4', dialect='ardupilotmega', baud=115200)
    mav_con_01.start() #Mavlink connection start

    #Set a connection for GC
    gc_connection = mavlinkplug.Modules.TcpConnection.TcpConnection(plug.plug_info(), ('', 17562), mav_con_01.ident())
    gc_connection.start()

    #Creating HIL environment
    hil_env = mavlinkplug.Modules.Hil.Hil(plug.plug_info(), mav_con_01.ident(), mavlinkplug.Modules.AircraftType.Plane, hil_sensor=False, quaternion=False)
    hil_env.add_thermal((43.6042600,1.4436700), 20, 10000)

    #Automatic Launch Procedure
    hil_env.start()

    #Server forever
    plug.server_forever() #Wait forever
