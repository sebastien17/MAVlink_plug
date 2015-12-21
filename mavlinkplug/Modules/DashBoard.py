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

import zmq
from flask import Flask, Response, render_template

#Parameters
ZQM_PORT_IN = "42017"


app = Flask(__name__, static_folder='statics', static_url_path='/statics')


#Defining incoming zmq communication
context = zmq.Context()


def event_stream():
    socket = context.socket(zmq.SUB)                  #0mq publisher
    socket.connect ("tcp://127.0.0.1:%s" % ZQM_PORT_IN)
    socket.setsockopt(zmq.SUBSCRIBE, '')
    while True:
        data = socket.recv()
        yield ('data: {0}\n\n'.format(data))

@app.route('/stream')
def sse_request():
    return Response(event_stream(), mimetype='text/event-stream')

@app.route('/')
def page():
    return render_template('sse.html')

if __name__ == '__main__':
    app.run()