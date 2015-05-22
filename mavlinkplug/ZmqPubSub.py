from __future__ import print
from zmq.eventloop import ioloop, zmqstream
import zmq, multiprocessing


class ZmqStream(multiprocessing.Process):
    """
    This is the base for all processes and offers utility functions
    for setup and creating new streams.
    """
    def __init__(self, zmq_context = None):
        super().__init__()
        if(zmq_context == None):
            self._context =  zmq.Context()
        else:
            self._context =  zmq_context
        self._loop = ioloop.IOLoop.instance()
    def stream(self, sock_type, addr, bind = True, callback=None, subscribe=b''):
        sock = self._context.socket(sock_type)
        if(isinstance(addr, str)):
            addr = addr.split(':')
        host, port = addr if len(addr) == 2 else (addr[0], None)
        if (bind):
            if port:
                sock.bind('tcp://%s:%s'.format(host, port))
            else:
                port = sock.bind_to_random_port('tcp://%s'.format(host))
        else:
            sock.connect('tcp://%s:%s' % (host, port))
        if sock_type == zmq.SUB:
            sock.setsockopt(zmq.SUBSCRIBE, subscribe)
        stream = zmqstream.ZMQStream(sock, self._loop)
        if (callback):
            stream.on_recv(callback)
        return stream
    def run(self):
        self._loop.start()
    def stop(self):
        self._loop.stop()
        
        
        
    