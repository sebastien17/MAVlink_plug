#!/usr/bin/env python
from __future__ import print_function

#Import 
import zmq
import sys
import logging
import threading
from pymavlink import mavutil
from time import sleep, time
from json import dumps, loads

class MAVLINK_Plug(object):
    
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
        self._verbose = False
        self._logging = False
        
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
                self.MAVLINK_connection(*self._connection_argv, **self._connection_kwargs)
            else:
                if msg is not None:
                    if (msg.get_type() != 'BAD DATA'):
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
                            if(self._logging):
                                logging.debug("{0} {1}".format(msg.get_type(), json_data))
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
            if(self._logging):
                logging.debug(string)
            self._count[2] += 1
            topic, messagedata = string.split(" ",1)
            cmd_dict = loads(messagedata)
            logging_string = '{0} : Not yet implemented'.format(cmd_dict['cmd'])
            if (topic == 'MAVLINK_CMD'):
                if (cmd_dict['cmd'] == 'RESET'):
                    self._mav.reboot_autopilot()
                    logging_string = 'Launch reboot_autopilot command'
                elif (cmd_dict['cmd'] == 'LOITER_MODE'):
                    self._mav.set_mode_loiter()
                    logging_string = 'Launch set_mode_loiter command'
                elif (cmd_dict['cmd'] == 'RTL_MODE'):
                    self._mav.set_mode_rtl()
                    logging_string = 'Launch set_mode_rtl command'
                elif (cmd_dict['cmd'] == 'MISSION_MODE'):
                    self._mav.set_mode_auto()
                    logging_string = 'Launch set_mode_auto command'
                elif(cmd_dict['cmd'] == 'WP_LIST_REQUEST'):
                    self._mav.waypoint_request_list_send()
                    logging_string = 'Launch waypoint_request_list_send command'
                elif(cmd_dict['cmd'] == 'WP_REQUEST'):
                    self._mav.waypoint_request_send(cmd_dict['seq'])
                    logging_string = 'Launch waypoint_request_send({0}) command'.format(cmd_dict['seq'])
                if(self._logging):
                    logging.debug(logging_string)          
        print('ZMQ_in loop stop')    
        
    def __del__(self):
        self._context.destroy()
        self.MAVLINK_in_ZMQ_out_close()
        self.ZMQ_in_close()
        logging.shutdown()
        
    def forever(self):
        while(True):
            try:
                sleep(1)
                if(self._verbose):
                    print('[MAVLINK IN, ZQM OUT, ZQM IN] : {0}'.format(self._count))
            except(KeyboardInterrupt, SystemExit):
                print('Exit called !!\nRecv/Send: {0}'.format(self._count))
                exit(0)
                
    def verbose(self, switch):
        if(switch):
            self._verbose = True
        else:
             self._verbose = False
             
    def logging(self, switch):
        if(switch):
            self._logging = True
        else:
             self._logging = False
    
        
        
if(__name__ == "__main__"):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--mavlink", type=str, help="define MAVLINK input", default="COM8")
    parser.add_argument("--baud", type=int, help="set baud rate if COM connection", default=57600)
    parser.add_argument("--dialect", type=str, help="define MAVLINK dialect", default="pixhawk")
    parser.add_argument("--zmq_out", type=int, help="define ZMQ port to publish MAVLINK data", default=42017)
    parser.add_argument("--zmq_in", type=int, help="define ZMQ port to suscribe to external commands", default=42018)
    parser.add_argument("--verbose", help="set verbose output", action="store_true")
    parser.add_argument("--logging", help="log DEBUG info in MAVlink_plug.log file", action="store_true")
    args = parser.parse_args()
 
    my_plug = MAVLINK_Plug()
    if(args.verbose):
        my_plug.verbose(True)
    if(args.logging):
        logging.basicConfig(filename='MAVlink_plug.log',level=logging.DEBUG,format='[%(levelname)s] %(asctime)s (%(threadName)-10s) %(message)s', filemode = 'w')
        my_plug.logging(True)
    my_plug.MAVLINK_connection( args.mavlink, baud=args.baud, dialect=args.dialect) # try to connect to MAVlink using mavutil.mvlink_connection parameters
    my_plug.MAVLINK_in_ZMQ_out(args.zmq_out)
    my_plug.ZMQ_in(args.zmq_in)
    my_plug.forever()

    