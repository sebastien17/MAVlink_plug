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
import zmq
from zmq.devices import ProcessDevice


class Plug(object):
    '''
    Plug class to bridge the signal.
    Create an instance zmq.ProcessDevice SUB -> QUEUE -> PUB
    SUB socket has no filter
    '''
    def __init__(self, zmq_in = None, zmq_out = None):
    '''
    Instanciate the object
    
    In :
        zmq_in  : zmq address:port combo for input (SUB)
        zmq_out  : zmq address:port combo for output (PUB)

    '''
        if(zmq_in == None):
            self.zmq_bridge_in = 'tcp://127.0.0.1:42569'
        else:
            self.zmq_bridge_in = zmq_in
        if(zmq_out == None):
            self.zmq_bridge_out = 'tcp://127.0.0.1:42568'        
        else:
            self.zmq_bridge_out = zmq_out
        self._processd = ProcessDevice(zmq.QUEUE, zmq.SUB, zmq.PUB)
        self._processd.bind_in(self.zmq_bridge_in)
        self._processd.bind_out(self.zmq_bridge_out)
        self._processd.setsockopt_in(zmq.SUBSCRIBE,b'')
    def start(self):
        self._processd.start()