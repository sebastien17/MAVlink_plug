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
    data_2 = '''{"id":"Vehicle","billboard":{"eyeOffset":{"cartesian":[0.0,0.0,0.0]},"horizontalOrigin":"CENTER","image":"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAADsMAAA7DAcdvqGQAAAEISURBVEhLvVXBDYQwDOuojHKj8LhBbpTbpBCEkZsmIVTXq1RVQGrHiWlLmTTqPiZBlyLgy/KSZQ5JSHDQ/mCYCsC8106kDU0AdwRnvYZArWRcAl0dcYJq1hWCb3hBrumbDAVMwAC82WoRvgMnVMDBnB0nYZFTbE6BBvdUGqVqCbjBIk3PyFFR/NU7EKzru+qZsau3ryPwwCRLKYOzutZuCL6fUmWeJGzNzL/RxAMrUmASSCkkAayk2IxPlwhAAYGpsiHQjbLccfdOY5gKkCXAMi7SscAwbQpAnKyctWyUZ6z8ja3OGMepwD8asz+9FnSvbhU8uVOHFIwQsI3/p0CfhuqCSQuxLqsN6mu8SS+N42MAAAAASUVORK5CYII=","pixelOffset":{"cartesian2":[0.0,0.0]},"scale":0.8333333333333334,"show":[{"boolean":true}],"verticalOrigin":"BOTTOM"},"position":{"cartographicDegrees":[0,0,100]}}'''
    x = 0
    y = 0
    z = 100
    while(running):
        if (turn == 0) :
            data = data_1
        elif(turn == 1):
            data = data_2
        else:
            (x,y,z) = (x+0.01, y, z)
            data = '{"id":"Vehicle","position":{ "cartographicDegrees":["'+ datetime.now().isoformat() +'",' + str(x) + ','+ str(y) + ',' + str(z) + ']}}'
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