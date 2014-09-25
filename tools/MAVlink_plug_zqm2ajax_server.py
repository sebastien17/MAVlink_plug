from urlparse import urlparse, parse_qs
import BaseHTTPServer
import SimpleHTTPServer
import zmq
import cgi
from json import dumps

AJAX_PORT = 43017
ZQM_PORT = "42017"
interface_file = ".\html_interface\index.html"

# Socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect ("tcp://*:%s" % ZQM_PORT)
topicfilter = ""


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
        topicfilter = postvars["topic"][0]
        # Subscribe to topic
        socket.setsockopt(zmq.SUBSCRIBE, topicfilter)
        string = socket.recv()
        topic, messagedata = string.split(" ",1)
        result = messagedata
        #Unsuscribe previous filter
        socket.setsockopt(zmq.UNSUBSCRIBE, topicfilter)
        self.wfile.write(result)

def start_server():
    """Start the server."""
    server_address = ("", AJAX_PORT)
    server = BaseHTTPServer.HTTPServer(server_address, TestHandler)
    server.serve_forever()

if __name__ == "__main__":
    start_server()