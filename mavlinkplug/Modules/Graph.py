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

# Constant
ENERGY_MSG_HEADER  = 'KE_PE'

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
        self._te = self._ke = self._pe = self._ie = 0.0

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
        self._nrg_data = np.zeros((4,1000), dtype=float)
        self._ke_plot = self._nrg_plot.plot(self._nrg_data[0], pen=(255,0,0), name="Kinetic Energy")
        self._pe_plot = self._nrg_plot.plot(self._nrg_data[1], pen=(0,255,0), name="Potential Energy")
        self._ie_plot = self._nrg_plot.plot(self._nrg_data[2], pen=(0,0,255), name="Internal Energy")
        self._te_plot = self._nrg_plot.plot(self._nrg_data[3], pen=(255,255,255), name="Total Energy")

        self._logging('Initializing')
    def _plug_2_graph(self, p_msg):
        plug_msg = mavlinkplug.Message.Message().unpack_from(p_msg[0])

        if(plug_msg.header.type == mavlinkplug.Message.TYPE.RAW.value and plug_msg.header.source == self._hil_ident) :
            #Processing Raw Message from HIL
            raw_data = [i for i in plug_msg.data.split(" ")]
            if(raw_data[0] == ENERGY_MSG_HEADER):
                # Processing Energy Raw Data Message
                self._ke = float(raw_data[1])
                self._pe = float(raw_data[2])
                self._te = self._ke + self._pe + self._ie
                self._update_graph()
        elif(plug_msg.header.type == mavlinkplug.Message.TYPE.KILL.value):
            self.stop()

    def update_graph(self):
        temp_array = np.array([self._ke,self._pe,self._ie,self._te]).reshape(4,1)
        self._nrg_data  = np.concatenate((self._nrg_data[:-1],temp_array),axis=1)
        self._ke_plot.setData(self._nrg_data[0])
        self._pe_plot.setData(self._nrg_data[1])
        self._ie_plot.setData(self._nrg_data[2])
        self._te_plot.setData(self._nrg_data[3])

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
