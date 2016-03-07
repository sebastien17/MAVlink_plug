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


from mavlinkplug.Base import ZmqBase
import mavlinkplug.Message
from time import sleep, time
import zmq

#Constant
ENERGY_MSG_HEADER  = 'KE_PE'


class Graph_Test(ZmqBase):

    def __init__(self, module_info):
        super(Graph_Test, self).__init__()
        self._addr_to_plug, self._addr_from_plug, self._ident = module_info
        self._stream2Plug = None
        self._streamFromPlug = None
        self._name = 'GraphTest'
        self._count = 0.0

    def setup(self):
        super(Graph_Test,self).setup()
        #Define stream listening from plug
        self._stream2Plug  = self.stream(zmq.PUB, self._addr_to_plug, bind = False)
        self._loop.call_later(1.0, self.beep)
        self._logging('Initializing')

    def beep(self):
        raw_string = "{0} {1} {2}".format(ENERGY_MSG_HEADER, self._count,self._count)
        energy_message = mavlinkplug.Message.RawData.build_full_message_from(mavlinkplug.Message.DESTINATION.ALL.value,
                                                                                 17,
                                                                                 long(time()),
                                                                                 raw_string)
        self._stream2Plug.send(energy_message.packed)


if(__name__ == '__main__'):


    import mavlinkplug.Modules.Graph
    import mavlinkplug.Plug

    # Creating plug
    plug = mavlinkplug.Plug.Plug()
    plug.start()

    graph_01 = mavlinkplug.Modules.Graph.Graph(plug.plug_info(), 17, 18)
    graph_01.start()

    graph_test_01 = Graph_Test(plug.plug_info())
    graph_test_01.start()

    #Server forever
    plug.server_forever() #Wait forever



