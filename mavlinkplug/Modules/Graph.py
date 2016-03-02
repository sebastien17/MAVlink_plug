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

#Core Module
from __future__ import print_function
from time import time
#External Module
import zmq
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np

#Internal Module
import mavlinkplug.Message
from mavlinkplug.Base import ZmqBase


class Graph(ZmqBase):
    def __init__(self, module_info, hil_ident, mav_ident,  name = None):
        super(Graph, self).__init__()
        self._addr_to_plug, self._addr_from_plug, self._ident = module_info
        self._stream2Plug = None
        self._streamFromPlug = None
        self._hil_ident = hil_ident
        self._mav_ident = mav_ident
        self._default_subscribe.append(mavlinkplug.Message.integer_pack(self._hil_ident))
        self._default_subscribe.append(mavlinkplug.Message.integer_pack(self._mav_ident))
        self._default_subscribe.append(mavlinkplug.Message.integer_pack(self._ident))

        if(name == None ):
            self._name = 'Graph_' + self._ident
        else:
            self._name = name
    def setup(self):
        super(Graph,self).setup()
        #Define stream listening from plug
        self._streamFromPlug = self.stream(zmq.SUB, self._addr_from_plug, bind = False, callback = self._plug_2_graph, subscribe = self._subscribe)
        self._stream2Plug  = self.stream(zmq.PUB, self._addr_to_plug, bind = False)

        #PyQt Graph Initialization
        self._win = pg.GraphicsWindow()
        self._win.setWindowTitle('MAvlinkPlug Graph Module')
        self._nrg_plot = self._win.addPlot()
        self._nrg_plot.enableAutoRangr('y',1.1)
        self._nrg_plot_data = dict()
        self._nrg_plot_data['E_Tot'] = self._nrg_plot_data['E_Chi'] = self._nrg_plot_data['E_Cin'] = self._nrg_plot_data['E_Pot'] = np.zeros(200, np.float16)

        self._logging('Initializing')
    def _plug_2_graph(self, p_msg):
        plug_msg = mavlinkplug.Message.Message().unpack_from(p_msg[0])

        if(plug_msg.header.type == mavlinkplug.Message.TYPE.RAW.value and plug_msg.header.source == self._hil_ident) :
            #Processing Raw Message from HIL
            raw_data = [i for i in plug_msg.data.split(" ")]
            if(raw_data[0] == 'KE_PE'):
                # Processing Energy Raw Data Message
                ke = float(raw_data[1])
                pe = float(raw_data[2])

        elif(plug_msg.header.type == mavlinkplug.Message.TYPE.KILL.value):
            self.stop()

    def stop(self):
        super(Graph, self).stop()
        self._logging('Closing')
    def _logging(self, msg, type = 'INFO'):
         logging_message = mavlinkplug.Message.LogData.build_full_message_from( 0,
                                                                                self._ident,
                                                                                long(time()),
                                                                                type+': '+ msg
                                                                                )
         self._stream2Plug.send(logging_message.packed)
