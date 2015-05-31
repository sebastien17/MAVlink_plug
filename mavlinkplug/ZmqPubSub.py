from __future__ import print_function
from zmq.eventloop import ioloop, zmqstream
import zmq, multiprocessing


class ZmqStream(multiprocessing.Process):
    """
    This is the base for all processes and offers utility functions
    for setup and creating new streams.
    """
    def __init__(self):
        super(ZmqStream,self).__init__()
        self._zmq_context =  zmq.Context()
        self._loop = None
    def setup(self):
        self._loop = ioloop.IOLoop.instance()
    def stream(self, sock_type, addr, bind = True, callback=None, subscribe=b''):
        sock = self._zmq_context.socket(sock_type)
        if (bind):
            sock.bind(addr)
        else:
            sock.connect(addr)
        if sock_type == zmq.SUB:
            sock.setsockopt(zmq.SUBSCRIBE, subscribe)
        stream = zmqstream.ZMQStream(sock, self._loop)
        if (callback):
            stream.on_recv(callback)
        return stream
    def run(self):
        self.setup()
        self._loop.start()
    def stop(self):
        self._loop.stop()
