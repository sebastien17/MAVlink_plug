from __future__ import print_function
'''Listener using pymavlink module
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

class MAVlink_ZMQ_Plug(object):
    
    def __init__(self):
        '''Initialization loop'''
        self._mav = None
        self._MAVLINK_in_ZMQ_out_running = False
        self._MAVLINK_in_ZMQ_out_thread = None
        self._ZMQ_in_running = False
        self._ZMQ_in_thread = None
        self._count = [0,0,0]
        self._connection_argv = None
        self._connection_kwargs = None
        self._context = zmq.Context()
        
    def MAVLINK_connection(self, *argv, **kwargs):
        '''Connection Manager'''
        self._connection_argv = argv        #Save connection parameters for eventual reconnect
        self._connection_kwargs = kwargs
        running = True
        print('Initiating mavlink connection', end="")
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
                
    def MAVLINK_in_ZMQ_out(self, zmq_port):
        '''MAVLINK_in_ZMQ_out manager'''
        if(self._MAVLINK_in_ZMQ_out_running):
            print('MAVLINK_in_ZMQ_out :: MAVLINK_in_ZMQ_out already running')
            return
        if( not isinstance( zmq_port, ( int, long ) )):
            print('MAVLINK_in_ZMQ_out :: ZMQ need to be an int : 0-65535')
            return
        self._MAVLINK_in_ZMQ_out_running = True
        self._MAVLINK_in_ZMQ_out_thread = threading.Thread(None, self._MAVLINK_in_ZMQ_out_loop, 'MAVLINK_in_ZMQ_out_loop',(zmq_port,))
        self._MAVLINK_in_ZMQ_out_thread.setDaemon(True)
        self._MAVLINK_in_ZMQ_out_thread.start()

    def MAVLINK_in_ZMQ_out_close(self):
        if(self._MAVLINK_in_ZMQ_out_running): #TODO : check if self._MAVLINK_in_ZMQ_out_thread is a thread instance
            self._MAVLINK_in_ZMQ_out_running = False
            self._MAVLINK_in_ZMQ_out_thread.join()
    
    def _MAVLINK_in_ZMQ_out_loop(self, zmq_port_in):
        print('MAVLINK_in_ZMQ_out loop start')
        socket = self._context.socket(zmq.PUB)                #0mq publisher
        socket.bind("tcp://*:%s" % zmq_port_in)
        while(self._MAVLINK_in_ZMQ_out_running):
            '''Listening loop'''
            try:
                msg = self._mav.recv_msg()            #Non blocking TBC
            except:
                self.connection(*self._connection_argv, **self._connection_kwargs)
            else:
                if msg is not None:
                    self._count[0] += 1
                    data = {}
                    for i in msg.get_fieldnames():
                        data[i]=msg.__dict__[i]
                    try:
                        json_data = dumps(data)
                    except:
                        pass                            #TODO : some smart logic to avoid : 'UnicodeDecodeError: 'utf8' codec can't decode byte 0xc9 in position 0: unexpected end of data'
                    else:
                        self._count[1] += 1
                        socket.send("{0} {1}".format(msg.get_type(), json_data))
        socket.close()
        print('MAVLINK_in_ZMQ_out loop stop')

    def ZMQ_in(self, zmq_port):
        '''ZMQ_in manager'''
        if(self._ZMQ_in_running):
            print('ZMQ_in :: ZMQ_in already running')
            return
        if( not isinstance( zmq_port, ( int, long ) )):
            print('ZMQ_in :: ZMQ need to be an int : 0-65535')
            return
        self._ZMQ_in_running = True
        self._ZMQ_in_thread = threading.Thread(None, self._ZMQ_in_loop, 'ZMQ_in_loop',(zmq_port,))
        self._ZMQ_in_thread.setDaemon(True)
        self._ZMQ_in_thread.start()

    def ZMQ_in_close(self):
        if(self._ZMQ_in_running): #TODO : check if self._ZMQ_in_thread is a thread instance
            self._ZMQ_in_running = False
            self._ZMQ_in_thread.join()
 
    
    def _ZMQ_in_loop(self, zmq_port):
        print('ZMQ_in loop start')
        socket = self._context.socket(zmq.SUB)                  #0mq publisher
        socket.bind("tcp://*:%s" % zmq_port)
        while(self._ZMQ_in_running):
            socket.setsockopt(zmq.SUBSCRIBE, 'CMD')            #MAVLINK PLUG command
            socket.setsockopt(zmq.SUBSCRIBE, 'MAVLINK_CMD')    #MAVLINK command
            string = socket.recv()
            self._count[2] += 1
            #self._mav.reboot_autopilot()
        print('ZMQ_in loop stop')    
        
    def __del__(self):
        self._context.destroy()
        self.MAVLINK_in_ZMQ_out_close()
        self.ZMQ_in_close()
        
    def forever(self):
        while(True):
            try:
                sleep(1)
                print(self._count)
            except(KeyboardInterrupt, SystemExit):
                print('Exit called !!\nRecv/Send: {0}'.format(self._count))
                exit(0)
        
        
if(__name__ == "__main__"):
    
    com_port = 'COM8'                                           # Get local machine COM port
    baud_rate=57600                                             # Set COM baud rate
    dialect_in_use="pixhawk"                                    # Dialect in use
    
    my_plug = MAVlink_ZMQ_Plug()
    my_plug.MAVLINK_connection(com_port, baud=baud_rate, dialect=dialect_in_use) # try to connect to MAVlink using mavutil.mvlink_connection parameters
    my_plug.MAVLINK_in_ZMQ_out(42017)
    my_plug.ZMQ_in(42018)
    my_plug.forever()

    