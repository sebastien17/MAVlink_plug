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


import subprocess, os, signal, telnetlib, time


JSBSIM_EXEC_NAME = 'JSBSim.exe'
JSBSIM_DEFAULT_ROOT_PATH =  JSBSIM_DEFAULT_PATH = os.path.dirname(__file__) + os.sep + '..' + os.sep + 'data' + os.sep
JSBSIM_DEFAULT_EXEC =  os.path.dirname(__file__) + os.sep + '..' + os.sep + 'data' + os.sep + 'bin' + os.sep + JSBSIM_EXEC_NAME


class JSBSimControl(object):
    input_port = 17133
    telnet_port = 17137
    output_port = 17139

    def __init__(self):
        self.jsbrootpath = JSBSIM_DEFAULT_ROOT_PATH
        self.jsbexec = JSBSIM_DEFAULT_EXEC
        self.aircraft = 'EasyStar'
        self.initfile = 'reset'
        self.defaultcommand = [self.jsbexec, '--realtime', '--nice', '--suspend', '--root=' + self.jsbrootpath, '--aircraft=' + self.aircraft, '--initfile=' + self.initfile]
        self.process_h = None
        self.jsb_in_tn = None
        self.jsb_in_address_port = ('127.0.0.1',self.telnet_port)

    def launch(self):
        self.process_h = subprocess.Popen(self.defaultcommand)
        time.sleep(2)
        self.jsb_in_tn = telnetlib.Telnet(self.jsb_in_address_port[0],self.jsb_in_address_port[1], 10)
        self.jsb_in_tn.read_until('JSBSim>')
        print(self.get('info'))
        print(self.get('help'))

    def send(self, data):
        self.jsb_in_tn.write(str(data) + '\n')

    def get(self, data):
        self.send(data)
        response = self.jsb_in_tn.read_until('JSBSim>')[1:-len('JSBSim>')]
        return response

    def resume(self):
        pass

    def pause(self):
        pass

    def stop(self):
        pass

    def terminate(self):
        self.send('quit')
        self.jsb_in_tn.close()
        os.kill(self.process_h.pid, signal.SIGTERM)

