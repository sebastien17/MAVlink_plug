#!/usr/bin/env python

import gevent.monkey
gevent.monkey.patch_all()

from datetime import datetime
from gevent import sleep
from gevent.pywsgi import WSGIServer
from flask import Flask, Response, render_template

app = Flask(__name__, static_folder='statics', static_url_path='/statics')

def event_stream():
    turn = 0
    running = True
    data_1 = '''{"id":"document","version":"1.0"}'''
    data_2 = '''{"id":"Vehicle","description": "Description test", "model":{"gltf":"statics/img/arrow.gltf","minimumPixelSize":50,"scale":1.0,"show":true}}'''
    x = 45
    y = 0
    z = 100
    while(running):
        if (turn == 0) :
            data = data_1
        elif(turn == 1):
            data = data_2
        else:
            (x,y,z) = (x+0.1, y, z)
            data = '{"id":"Vehicle","position":{ "cartographicDegrees":["'+ datetime.now().isoformat() +'",' + str(x) + ','+ str(y) + ',' + str(z) + ']},"_information":{"cartographicDegrees":["'+ datetime.now().isoformat() +'",' + str(x) + ','+ str(y) + ',' + str(z) + ']}}'
        turn += 1
        sleep(0.1)
        yield ('event:czml\ndata: {0}\n\n'.format(data))

@app.route('/stream')
def sse_request():
    return Response(event_stream(), mimetype='text/event-stream')

@app.route('/')
def page():
    return render_template('czml.html')

if __name__ == '__main__':
    http_server = WSGIServer(('127.0.0.1', 1717), app)
    http_server.serve_forever()