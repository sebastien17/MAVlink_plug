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


import subprocess, os, signal, telnetlib, time, socket


JSBSIM_EXEC_NAME = 'JSBSim.exe'
JSBSIM_DEFAULT_ROOT_PATH =  JSBSIM_DEFAULT_PATH = os.path.dirname(__file__) + os.sep + '..' + os.sep + 'data' + os.sep
JSBSIM_DEFAULT_EXEC =  os.path.dirname(__file__) + os.sep + '..' + os.sep + 'data' + os.sep + 'bin' + os.sep + JSBSIM_EXEC_NAME


class JSBSimControl(object):
    def __init__(self, aircraft='EasyStar', initfile='reset', input_port=17133, telnet_port=17137, output_port=17139, address = '127.0.0.1'):
        self.jsbrootpath = JSBSIM_DEFAULT_ROOT_PATH
        self.jsbexec = JSBSIM_DEFAULT_EXEC
        self.aircraft = aircraft
        self.initfile = initfile
        self.defaultcommand = [self.jsbexec,
                               '--realtime',
                               '--nice',
                               '--suspend',
                               '--root=' + self.jsbrootpath,
                               '--aircraft=' + self.aircraft,
                               '--initfile=' + self.initfile
                               ]
        self.process_h = None
        self.jsb_in_tn = None
        self.address = address
        self.telnet_port = telnet_port
        self.input_port = input_port
        self.output_port = output_port
        # Socket initialization
        self.socket_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket_in.bind((self.address, self.input_port))
        self.socket_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket_out.setblocking(0)
        self.socket_out.bind((self.address, self.output_port))

    def launch(self):
        self.process_h = subprocess.Popen(self.defaultcommand)
        time.sleep(2)
        self.jsb_in_tn = telnetlib.Telnet(self.address,self.telnet_port, 10)
        self.jsb_in_tn.read_until('JSBSim>')

    def _send(self, data):
        self.jsb_in_tn.write(str(data) + '\n')

    def set(self, list):
        for key,val in list.items():
            self._send("set {0} {1}".format(key,val))

    def get(self, data):
        self._send(data)
        response = self.jsb_in_tn.read_until("JSBSim>")[1:-len("JSBSim>")]
        return response

    def resume(self):
        self._send("resume")

    def hold(self):
        self._send("hold")
        
    def iterate(self,num):
        self._send("iterate {0}".format(num))

    def terminate(self):
        self._send("quit")
        self.jsb_in_tn.close()
        os.kill(self.process_h.pid, signal.SIGTERM)
