#!/usr/bin/env python

#import gevent
from gevent.pywsgi import WSGIServer
import zmq.green as zmq

from flask import Flask, request, Response, render_template

#Parameters
ZQM_PORT_IN = "42017"


app = Flask(__name__, static_folder='statics', static_url_path='/statics')


#Defining incoming zmq communication
context = zmq.Context()
socket = context.socket(zmq.SUB)                  #0mq publisher
socket.connect ("tcp://127.0.0.1:%s" % ZQM_PORT_IN)
socket.setsockopt(zmq.SUBSCRIBE, '')

def event_stream():
    while True:
        data = socket.recv()
        yield ('data: {0}\n\n'.format(data))

@app.route('/stream')
def sse_request():
    return Response(
            event_stream(),
            mimetype='text/event-stream')

@app.route('/')
def page():
    return render_template('sse.html')

if __name__ == '__main__':
    http_server = WSGIServer(('127.0.0.1', 1717), app)
    http_server.serve_forever()