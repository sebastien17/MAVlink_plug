from urlparse import urlparse, parse_qs
import BaseHTTPServer
import SimpleHTTPServer
import zmq
import cgi
from json import dumps, loads
import threading

AJAX_PORT = 43017
ZQM_PORT_IN = "42017"
ZQM_PORT_OUT = "42018"
interface_file = ".\html_interface\index.html"
running = False
data = {}

context = zmq.Context()
socket_out = context.socket(zmq.PUB)
socket_out.connect("tcp://127.0.0.1:%s" % ZQM_PORT_OUT)

class TestHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
    """The test example handler."""
    
    def do_POST(self):
        """Handle a post request by returning the square of the number."""
        ctype, pdict = cgi.parse_header(self.headers.getheader('content-type'))
        if ctype == 'multipart/form-data':
            postvars = cgi.parse_multipart(self.rfile, pdict)
        elif ctype == 'application/x-www-form-urlencoded':
            length = int(self.headers.getheader('content-length'))
            postvars = cgi.parse_qs(self.rfile.read(length), keep_blank_values=1)
        else:
            postvars = {}
        messagedata = None
        type = postvars["type"][0]
        topic = postvars["topic"][0]
        
        #GET_VALUE type
        if(type == 'GET_VALUE'):                        
            if(topic in data):
                messagedata = dumps(data[topic])
            else:
                messagedata = dumps({})
            self.wfile.write(messagedata)
        else:
            #MAVLINK_CMD type
            if(type == 'MAVLINK_CMD'):                  
                socket_out.send('MAVLINK_CMD {0}'.format(topic))

def start_server():
    """Start the server."""
    server_address = ("", AJAX_PORT)
    server = BaseHTTPServer.HTTPServer(server_address, TestHandler)
    server.serve_forever()

def ZMQ_suscriber_thread():
    print('ZMQ_in loop start')

    socket = context.socket(zmq.SUB)                  #0mq publisher
    socket.connect ("tcp://127.0.0.1:%s" % ZQM_PORT_IN)
    socket.setsockopt(zmq.SUBSCRIBE, '')
    while(running):
        string = socket.recv()
        topic, messagedata = string.split(" ",1)
        if(topic not in data):
            data[topic] = {}
        data[topic] = loads(messagedata)
    print('ZMQ_in loop stop') 
    
    
def start_ZMQ_suscriber():
    global running
    running = True
    ZMQ_thread = threading.Thread(None, ZMQ_suscriber_thread, 'ZMQ_thread',)
    ZMQ_thread.setDaemon(True)
    ZMQ_thread.start()

if __name__ == "__main__":
    start_ZMQ_suscriber()
    start_server()
    