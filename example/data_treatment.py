#!/usr/bin/env python
# -*- coding: utf-8 -*-

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

LOG_FILE = "Test_HIL.log"
OUTPUT_FILE = "Output.log"
LIST_TO_REMOVE = ["{time_usec : ", " roll : ", " pitch : ", " yaw : ", " rollspeed : ", " pitchspeed : ", " yawspeed : ", " lat : ", " lon : ",  "alt :", " vx : ", " vy : ", " vz : ", " xacc : ", " yacc : ", " zacc : ", "}"]

with open(OUTPUT_FILE, 'w') as outfile, open(LOG_FILE, 'r') as infile:
    for line in infile:
        if 'HIL_STATE {' in line :
            temp = line.split('\t',5)
            text = temp[-1][len('HIL_STATE '):]
            for str2repl in LIST_TO_REMOVE:
                text = text.replace(str2repl,"")
            outfile.write(text)

