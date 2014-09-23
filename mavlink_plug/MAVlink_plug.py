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
from pymavlink import mavutil
from time import sleep, time
from json import dumps

def loop(mav):
    ''' Main loop for MAVlink plug '''
    running = True
    
    port = "42017"                      #0mq publication port def
    context = zmq.Context()             #0mq context initialization 
    socket = context.socket(zmq.PUB)    #0mq publisher
    socket.bind("tcp://*:%s" % port)    #0mq publication port set
    
    data = dict()                       #Data storage
    t0 = time()
    while(running):
        '''Listening loop'''
        msg = mav.recv_msg()            #Non blocking TBC
        if msg is not None:
            if msg.get_type() not in data:
                data[msg.get_type()] = dict()
            for i in msg.get_fieldnames():
                data[msg.get_type()][i]=msg.__dict__[i]
        if ((time() - t0) > 1 ):
            for item in data:
                json_data = dumps(data[item])
                print("{0} {1}".format(item, json_data))
                socket.send("{0} {1}".format(item, json_data))
            t0 = time()

def connection(*argv, **kwargs):
    '''Connection Loop '''
    running = True
    print('Initiating mavlink connection')
    print('Try', end="")
    while(running):
        print('.', end="")
        try:
            mav = mavutil.mavlink_connection(*argv,**kwargs)
        except:
            pass
        else:
            print('!')
            print('COM Connection OK')
            loop(mav)
            running = False # If successful one time ==> leave
        sleep(1)

        
if(__name__ == "__main__"):
    com_port = 'COM8'                                           # Get local machine COM port
    baud_rate=57600                                             # Set COM baud rate
    dialect_in_use="pixhawk"                                    # Dialect in use
    connection(com_port, baud=baud_rate, dialect=dialect_in_use)   # try to connect to MAVlink
