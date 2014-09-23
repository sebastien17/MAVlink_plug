from urlparse import urlparse, parse_qs
import BaseHTTPServer
import SimpleHTTPServer
import zmq

AJAX_PORT = 43017
ZQM_PORT = "42017"

# Socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect ("tcp://localhost:%s" % ZQM_PORT)
topicfilter = ""


class TestHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
    """The test example handler."""
    
    def do_POST(self):
        """Handle a post request by returning the square of the number."""
        length = int(self.headers.getheader('content-length'))        
        data_string = self.rfile.read(length)
        
        # Subscribe to HEARBEAT topic
        topicfilter = data_string
        socket.setsockopt(zmq.SUBSCRIBE, topicfilter)
        try:
            string = socket.recv()
            topic, messagedata = string.split(" ",1)
            result = messagedata
        except:
            result = 'error'
        self.wfile.write(result)
        
    def do_GET(self):
        """Handle a post request by returning the square of the number."""
        query_components = parse_qs(urlparse(self.path).query)

        topicfilter = query_components["topic"][0]
        # Subscribe to topic
        socket.setsockopt(zmq.SUBSCRIBE, topicfilter)
        try:
            string = socket.recv()
            topic, messagedata = string.split(" ",1)
            result = messagedata
        except:
            result = 'error'
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