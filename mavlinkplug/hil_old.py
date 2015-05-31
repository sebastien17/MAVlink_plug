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

import mavlinkplug, pyfdm
from pymavlink.dialects.v10 import pixhawk as mavlink
import zmq, time, os
from json import loads

#Define default path for package included data
DATA_PATH = os.path.dirname(__file__)+os.sep+'data'+os.sep

#Method added to Plug class
def __function_to_add(self, *argv, **kwargs):
    ident = '{0:02d}'.format(len(self._input_list))
    h = mavlinkplug.hil.hil(self._zmq_context, self._zmq_bridge_out, ident, *argv, **kwargs)
    h.run()
    self._input_list.append(h)
    return h

#Add hil method to Plug class (defined by __function_to_add)
mavlinkplug.Plug.plugin_register('hil', __function_to_add)
    
class hil(mavlinkplug.ModBase):
    '''
    Module for HIL simulation
    Translate information from MAVLINK Connection to  FL and back
    '''
    def __init__(self, zmq_context, from_plug_zmq_sock, ident, mav_connection, to_FL_zmq_sock_port, from_FL_zmq_sock_port,FL_instance):
        super(hil, self).__init__()
        self._zmq_context = zmq_context
        self._from_plug_zmq_sock = from_plug_zmq_sock
        self._ident = ident
        self._mav = mav_connection
        self._to_FL_zmq_sock_port = to_FL_zmq_sock_port
        self._from_FL_zmq_sock_port = from_FL_zmq_sock_port
        self._out_msg = 0
        self._in_msg = 0
        #ZMQ sockets initialization
        self._from_plug_zmq_socket = self._zmq_context.socket(zmq.SUB)
        self._from_plug_zmq_socket.connect(self._from_plug_zmq_sock)                                                #Connect to bridge output
        self._from_plug_zmq_socket.setsockopt(zmq.SUBSCRIBE, mavlinkplug.ZMQ_MESSAGE_JSON + self._mav.ident())      #Filter on encrypted message with mav ident
        self._to_FL_zmq_socket = self._zmq_context.socket(zmq.PUB)
        self._to_FL_zmq_socket.bind(self._to_FL_zmq_sock_port)                                                       
        self._from_FL_zmq_socket = self._zmq_context.socket(zmq.SUB)
        self._from_FL_zmq_socket.bind(self._from_FL_zmq_sock_port)                                          
        self._from_FL_zmq_socket.setsockopt(zmq.SUBSCRIBE, '')                                                  #No filters
        self._FL_instance = FL_instance
        self._hardware_ready = False
        self._FL_ready = False
        print('HIL instance {0} initialized'.format(self._ident))
    
    def hardware_initialize(self):
        while(self._mav.mav_handle() == None):
            time.sleep(1)
        self._mav.mavlink_command('SET_HIL_ARM')
        #TODO : add check
        time.sleep(2)
        self._mav.mavlink_command('RESET')
        #TODO : add check

    def _mod(self):
        #Need 2 threads
        self._mav_2_FL()
        self._FL_2_mav()

    @mavlinkplug.in_thread(True)
    def _mav_2_FL(self):
        string = self._from_plug_zmq_socket.recv()
        msg = QuadCopter.mav_2_FL(string)
        if(msg !=  None):
            self._to_FL_zmq_socket.send(' '.join(*msg) 

    @mavlinkplug.in_thread(True)
    def _FL_2_mav(self):
        pass
    def info(self):
        pass
        #return {'ident' :  self._ident, 'address': self._address, 'msg_stats': {'out_msg': self._out_msg,'in_msg': self._in_msg, 'connection activated': self._connection_activated}}



if __name__ == "__main__":
    pass