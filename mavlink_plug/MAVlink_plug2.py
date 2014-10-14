from __future__ import print_function

import gevent
import zmq.green as zmq
from pymavlink import mavutil

import sys
import logging
from json import dumps, loads



class MAVLINK_Plug(object):
    
    def __init__(self):
        self._zmq_context = zmq.Context()
        self._bridge_h = gevent.spawn(self._bridge)
        self.prefix_string = 'M_'
        self._spawn_in_h = []
        self._spawn_out_h = []
    
    def _bridge(self):
        ''' Create the ZMQ bridge between in & out'''
        print('Creating in process bridge')
        socket_in =  self._zmq_context.socket(zmq.SUB)
        socket_in.bind('inproc://in')                           #Bind because most stable
        socket_in.setsockopt(zmq.SUBSCRIBE, '')
        socket_out =  self._zmq_context.socket(zmq.PUB)
        socket_out.bind('inproc://out')                         #Bind because most stable
        while (True):
            string = socket_in.recv()                           #Yield to event loop
            print(string)
            logging.debug(string)
            socket_out.send(string)
        
    def MAVLINK_Connection_In(self,*argv, **kwargs):
        ''' Create a new MAVLINK connection using mavutil.mavlink_connection'''
        connection_in_number = len(self._spawn_in_h)
        def try_connection(*argv, **kwargs):
            print('Initialising MAVLINK connection')
            while(True):
                try:
                    result = mavutil.mavlink_connection(*argv,**kwargs) # TODO : monkey_patch
                except:
                    gevent.sleep(1)                             #Wait 1 second until next try
                    pass
                else:
                    break
            print('MAVLINK connection acquired')
            return result
        
        def infinite_listening(number, *argv, **kwargs):
            socket_out =  self._zmq_context.socket(zmq.PUB)
            socket_out.connect('inproc://in')                      #New socket which publish to the bridge
            mav = try_connection(*argv, **kwargs)
            ident = '{0}{1}'.format(self.prefix_string,number)
            while(True):
                try:
                    msg = mav.recv_msg()                        #Blocking TBC
                except:
                    print('MAVLINK connection lost')
                    mav = try_connection(*argv, **kwargs)
                else:
                    if msg is not None:
                        if (msg.get_type() != 'BAD DATA'):
                            data = {}
                            d_type = msg.get_type()
                            data[d_type] = {}
                            for i in msg.get_fieldnames():
                                data[d_type][i]=msg.__dict__[i]
                            try:
                                json_data = dumps(data)
                            except:
                                pass                                #TODO : some smart logic to avoid : 'UnicodeDecodeError: 'utf8' codec can't decode byte 0xc9 in position 0: unexpected end of data'
                            else:
                                string = "{0} {1}".format(ident, json_data)
  
                                socket_out.send(string)
                                logging.debug(string)
                gevent.sleep(0.01)
        self._spawn_in_h.append(gevent.spawn(infinite_listening,connection_in_number,*argv, **kwargs))


        
        
        
if(__name__ == "__main__"):
    logging.basicConfig(filename='MAVlink_plug.log',level=logging.DEBUG,format='[%(levelname)s] %(asctime)s (%(threadName)-10s) %(message)s', filemode = 'w')
    my_plug = MAVLINK_Plug()
    my_plug.MAVLINK_Connection_In('COM8', baud=57600, dialect='pixhawk')
    gevent.wait()












