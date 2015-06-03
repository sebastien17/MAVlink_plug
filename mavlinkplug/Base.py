from __future__ import print_function
from zmq.eventloop import ioloop, zmqstream
import zmq, multiprocessing, threading

def in_thread(isDaemon = True):
    def base_in_thread(fn):
        '''Decorator to create a threaded function '''
        def wrapper(*args, **kwargs):
            t = threading.Thread(target=fn, args=args, kwargs=kwargs)
            t.setDaemon(isDaemon)
            t.start()
            return t
        return wrapper
    return base_in_thread

class MAVLinkPlugZmqBase(multiprocessing.Process):
    """
    This is the base for all processes and offers utility functions
    for setup and creating new streams.
    """
    def __init__(self, zmq_context = None):
        super(ZmqBase,self).__init__()
        self._zmq_context =  zmq_context
        self._loop = None
    def setup(self):
        if(self._zmq_context == None):
            self._zmq_context = zmq.Context()
        self._loop = ioloop.IOLoop.instance()
    def stream(self, sock_type, addr, bind = True, callback=None, subscribe=b''):
        sock = self._zmq_context.socket(sock_type)
        if (sock_type == zmq.SUB):
            sock.setsockopt(zmq.SUBSCRIBE, subscribe)
        if (bind):
            sock.bind(addr)
        else:
            sock.connect(addr)
        _stream = zmqstream.ZMQStream(sock, self._loop)
        _stream.flush(zmq.POLLIN|zmq.POLLOUT)
        if (callback):
            _stream.on_recv(callback)
        return _stream
    def run(self):
        self.setup()
        self._loop.start()
    def stop(self):
        self._loop.stop()

        
class MAVLinkPlugModBase(object):
    ''' Base class for all mods'''
    def __init__(self):
        self._run = False
        self._thread = None
        self.isDaemon = True
        self._ident = ''
    def _mod(self):
        pass
    def run(self):
        self._run = True
        @in_thread(self.isDaemon)
        def __t():
            self._mod()
        self._thread = __t()
    def stop(self):
        self._run = False
    def ident(self):
        '''Return connection ident'''
        return self._ident
    def info(self):
        '''Return connection information'''
        return {}