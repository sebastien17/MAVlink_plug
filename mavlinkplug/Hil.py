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
import mavlinkplug.Base
import zmq
from time import sleep

class Hil(mavlinkplug.Base.ZmqBase):
    def __init__(self, Aircraft_Type_cls, Plug_inst):
        super(Hil, self).__init__(Plug_inst.Context())
        self._addr_from_plug = Plug_inst.zmq_bridge_out
        self._addr_to_plug = Plug_inst.zmq_bridge_in
        self._addr_to_FL = 'tcp://127.0.0.1:45063'
        self._addr_from_FL = 'tcp://127.0.0.1:45064'
        self._Aircraft_Type_cls = Aircraft_Type_cls
        self.daemon = True
    def setup(self):
        super(Hil,self).setup()
        #Define stream listening from plug
        self.stream(zmq.SUB, self._addr_from_plug, bind = False, callback = self._plug_2_FL)
        #Define stream publishing to FL
        self._stream_to_FL  = self.stream(zmq.PUB, self._addr_to_FL)
        #Define stream listening from FL
        self.stream(zmq.SUB, self._addr_from_FL, callback = self._FL_2_plug)
        #Define stream publishing to plug
        self._stream_to_plug  = self.stream(zmq.PUB, self._addr_to_plug, bind = False)
    def hardware_initialize(self):
        # while(self._Mav_inst.mav_handle() == None):
            # time.sleep(1)
        # self._Mav_inst.mavlink_command('SET_HIL_ARM')
        #TODO : add check
        # time.sleep(2)
        # self._Mav_inst.mavlink_command('RESET')
        #TODO : add check
        pass
    def FL_initialize(self):
        aircraft = self._Aircraft_Type_cls(zmq_in = self._addr_to_FL, zmq_out = self._addr_from_FL)
        aircraft.start()
    def _plug_2_FL(self, msg):
        _msg = msg[0]
        data_2_FL = self.Aircraft_Type_cls.mav_2_FL(_msg)
        if(data_2_FL != None):
            msg_2_FL = ' '.join(data_2_FL)
            self._stream_to_FL(msg_2_FL)
    def _FL_2_plug(self, msg):
        _msg = msg[0]
        data_2_plug = self.Aircraft_Type_cls.FL_2_mav(_msg)
