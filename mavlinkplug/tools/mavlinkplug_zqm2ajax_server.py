from urlparse import urlparse, parse_qs
import BaseHTTPServer
import SimpleHTTPServer
import zmq
import cgi
from json import dumps, loads
import threading
from time import time, sleep


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
        messagedata = dumps({})
        type = postvars["type"][0]
        topic = postvars["topic"][0]
        
        #GET_VALUE topic
        if(topic == 'GET_VALUE'):                        
            if(type in data):
                temp_data = data[type].copy()
                temp_data['_dt'] = time() - temp_data['_ts'] #Adding delta time value to data
                messagedata = dumps(temp_data)
                del(temp_data)
        elif(topic == 'MAVLINK_CMD'):
            #MAVLINK_CMD topic                 
            if( type == 'RESET' or type == 'LOITER_MODE' or type == 'RTL_MODE' or type == 'MISSION_MODE' ):
                cmd_dict = {'cmd': type}
                socket_out.send('{0} {1}'.format(topic, dumps(cmd_dict)))
            elif(type == 'WP_LIST_REQUEST'):
                data['MISSION_ITEM'] = {}    # reset waypoint list in server
                start_time = time()
                cmd_dict = {'cmd': type}
                socket_out.send('{0} {1}'.format(topic, dumps(cmd_dict)))
                sleep(1)            #TODO : some thing more elegant
                if('MISSION_COUNT' in data):
                    if(data['MISSION_COUNT']['count'] > 0 and data['MISSION_COUNT']['_ts'] > start_time):
                        start_time = time()
                        for i in range(data['MISSION_COUNT']['count']):
                            cmd_dict = {'cmd': 'WP_REQUEST', 'seq': i}
                            socket_out.send('MAVLINK_CMD {0}'.format(dumps(cmd_dict)))
                        sleep(3)
                        if(len(data['MISSION_ITEM']) == data['MISSION_COUNT']['count']):
                            messagedata = dumps(data['MISSION_ITEM']) 
        self.wfile.write(messagedata)
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
        if(topic == 'BAD_DATA'):
            continue
        # if(topic not in data):
            # data[topic] = {}
        if(topic == 'MISSION_ITEM'):
            '''MISSION ITEM treatment'''
            mission_item_data = loads(messagedata)
            data['MISSION_ITEM'][int(mission_item_data['seq'])] = loads(messagedata)
            data['MISSION_ITEM'][int(mission_item_data['seq'])]['_ts'] = time() 
        else:
            data[topic] = loads(messagedata)
            data[topic]['_ts'] = time()                      #timestamp
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
    