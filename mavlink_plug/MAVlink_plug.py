from __future__ import print_function
'''COM Listener using pymavlink module
    Important msg methods :
    'get_crc'               :
    'get_fieldnames'        : list of fieldname [(str)]
    'get_header'            :
    'get_msgId'             :
    'get_msgbuf'            : raw message (str)
    'get_payload'           : 
    'get_seq'               :
    'get_srcComponent'      :
    'get_srcSystem'         :
    'get_type'              : type of message (e.g HEARTBEAT) (str)
    'to_dict'               : ???
    'to_json'               : ???
    parameter are directly available see fieldnames 
'''

#Import 
import zmq
import sys
import logging
import threading
from pymavlink import mavutil
from time import sleep, time
from json import dumps

class ZQM_Plug(object):
    
    def __init__(self):
        '''Initialization loop'''
        self._mav = None
        self._data = {}
        self._data_lock = threading.RLock()
        self._listener_running = False
        self._listener_thread = None
        self._publisher_running = False
        self._publisher_thread = None
        self._frequency = 25.0 #Default frequency = 25 Hz
        self._zqm_port = 42017
        self._count = [0,0,0]
        self._connection_argv = None
        self._connection_kwargs = None
        
    def _listening_loop(self):
        '''Internal Listening loop'''
        while(self._listener_running):
            '''Listening loop'''
            try:
                msg = self._mav.recv_msg()            #Non blocking TBC
            except:
                self.connection(*self._connection_argv, **self._connection_kwargs)
            else:
                if msg is not None:
                    self._data_lock.acquire()
                    if msg.get_type() not in self._data:
                        self._data[msg.get_type()] = dict()
                    for i in msg.get_fieldnames():
                        self._data[msg.get_type()][i]=msg.__dict__[i]
                    self._data_lock.release()
                    self._count[0] += 1
                    print(self._count)
                    
    def listener(self,running=True):
        '''Listen manager'''
        if(running):
            self._listener_running = True
            self._listener_thread = threading.Thread(None, self._listening_loop, 'MAVLINK_Listener')
            self._listener_thread.setDaemon(True)
            self._listener_thread.start()
        else:
            self._listener_running = False
            self._listener_thread.join()
    
    def _publishing_loop(self):
        context = zmq.Context()                         #0mq context initialization 
        socket = context.socket(zmq.PUB)                #0mq publisher
        socket.bind("tcp://*:%s" % self._zqm_port)      #0mq publication port set
        while(self._publisher_running):
            self._data_lock.acquire()
            for item in self._data:
                try:
                    json_data = dumps(self._data[item])
                except(UnicodeDecodeError):
                    continue                            #TODO : some smart logic to avoid : 'UnicodeDecodeError: 'utf8' codec can't decode byte 0xc9 in position 0: unexpected end of data'
                self._count[2] += 1
                socket.send("{0} {1}".format(item, json_data))
            self._data_lock.release()
            self._count[1] += 1
            print(self._count)
            sleep(1/self._frequency)
    
    def set_zqm_port(self, port):
        self._zqm_port = port
        
    def get_data(self):
        return self._data
    
    def publisher(self, running=True):
        '''ZQM Publisher''' 
        if(running):
            if(self._zqm_port == 0):
                print('ZQM port definition error!!!')
            else:
                self._publisher_running = True
                self._publisher_thread = threading.Thread(None, self._publishing_loop, 'ZQM_publisher')
                self._publisher_thread.setDaemon(True)
                self._publisher_thread.start()
        else:
            self._publisher_running = False
            self._publisher_thread.join()
        
    def connection(self, *argv, **kwargs):
        '''Connection Manager'''
        self._connection_argv = argv
        self._connection_kwargs = kwargs
        running = True
        print('Initiating mavlink connection')
        while(running):
            print('.', end="")
            try:
                self._mav = mavutil.mavlink_connection(*argv,**kwargs)
            except:
                sleep(1)    #Wait 1 second until next try
                pass
            else:
                running = False
                print('!')
                
    def forever(self):
        while(True):
            try:
                sleep(1)
            except(KeyboardInterrupt, SystemExit):
                print('Exit called !!')
                exit(0)

    def __del__(self):
        self.listener(False)
        self.publisher(False)
        
if(__name__ == "__main__"):
    
    com_port = 'COM8'                                           # Get local machine COM port
    baud_rate=57600                                             # Set COM baud rate
    dialect_in_use="pixhawk"                                    # Dialect in use
    
    my_plug = ZQM_Plug()
    my_plug.connection(com_port, baud=baud_rate, dialect=dialect_in_use) # try to connect to MAVlink using mavutil.mvlink_connection parameters
    my_plug.listener()
    my_plug.publisher()
    my_plug.forever()

    