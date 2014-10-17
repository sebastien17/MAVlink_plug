#!/usr/bin/env python
from __future__ import print_function

#Import 
import zmq
import logging
import threading
from pymavlink import mavutil
from time import sleep, time
from json import dumps, loads

#Threading decorator definition
def in_thread(fn):
    def wrapper(*args, **kwargs):
        t = threading.Thread(target=fn, args=args, kwargs=kwargs)
        t.setDaemon(True)
        t.start()
        return t
    return wrapper

    
class MAVLINK_Plug(object):
    
    def __init__(self, prefix):
        self._zmq_context = zmq.Context()
        self._bridge_thread = self._bridge()
        self._thread_mav_dict = {}                              #Thread list for MAVLINK connection
        self._thread_other_dict = {}                            #Thread list for other than MAVLINK connection
        self._count = [0,0,0,0]                                 #Message  count
        self._verbose = False
        self._logging = False
        self._prefix_string = prefix
        
    def __del__(self):
        self._zmq_context.destroy()
        logging.shutdown()

    @in_thread
    def _bridge(self):
        ''' Create the ZMQ bridge between in & out'''
        self._print('Creating in process bridge')
        socket_in =  self._zmq_context.socket(zmq.SUB)
        socket_in.bind('inproc://in')                           #Bind because most stable
        socket_in.setsockopt(zmq.SUBSCRIBE, '')
        socket_out =  self._zmq_context.socket(zmq.PUB)
        socket_out.bind('inproc://out')                         #Bind because most stable
        while (True):
            string = socket_in.recv()                           #Yield to event loop
            self._count[2] += 1
            socket_out.send(string)
    
    @in_thread
    def MAVLINK_connection(self,*argv, **kwargs):
        connection_in_number = len(self._thread_mav_dict)        #Define the number of the connection
        ident = '{0}{1:02d}'.format(self._prefix_string,connection_in_number)
        self._thread_mav_dict[ident] = {'thread' : threading.currentThread(), 'run' : True }     #Auto registration of the thread
        
        def try_connection():
            self._print('Initialising MAVLINK connection {0}'.format(ident))
            while(self._thread_mav_dict[ident]['run']):
                try:
                    result = mavutil.mavlink_connection(*argv,**kwargs) # TODO : monkey_patch
                except:
                    sleep(1)                                #Wait 1 second until next try
                else:
                    break
            self._print('MAVLINK connection {0} acquired'.format(ident))
            return result
        
        socket =  self._zmq_context.socket(zmq.PUB)
        socket.connect('inproc://in')                       #New socket which publish to the bridge
        self._thread_mav_dict[ident]['mav_h'] = try_connection()
        
        self._print('MAVLINK_connection {0} loop start'.format(ident))
        while(self._thread_mav_dict[ident]['run']):
            try:
                msg = self._thread_mav_dict[ident]['mav_h'].recv_msg()                        #Blocking TBC
            except:
                self._print('MAVLINK connection {0} lost'.format(ident))
                self._thread_mav_dict[ident]['mav_h'] = try_connection()
            else:
                if msg is not None:
                    if (msg.get_type() != 'BAD DATA'):      #BAD DATA message ignored
                        self._count[0] += 1
                        data = {}
                        d_type = msg.get_type()
                        data[d_type] = {}
                        for i in msg.get_fieldnames():
                            data[d_type][i]=msg.__dict__[i]
                        try:
                            json_data = dumps(data)
                        except:
                            pass                            #TODO : some smart logic to avoid : 'UnicodeDecodeError: 'utf8' codec can't decode byte 0xc9 in position 0: unexpected end of data'
                        else:
                            string = "{0} {1}".format(ident, json_data)
                            self._count[1] += 1
                            socket.send(string)
                            logging.debug(string)
        socket.close()
        self._print('MAVLINK_connection {0} loop stop'.format(ident))
    
    @in_thread
    def ZMQ_publisher(self, port):
        connection_out_number = len(self._thread_other_dict)                  #Define the number of the connection
        ident = '{0:02d}'.format(connection_out_number)
        self._thread_other_dict[ident] = {'thread' : threading.currentThread(), 'run' : True }     #Auto registration of the thread

        socket_in = self._zmq_context.socket(zmq.SUB)
        socket_in.connect('inproc://out')                           #Connect to bridge output
        socket_in.setsockopt(zmq.SUBSCRIBE, '')                     #No filter
        socket_out = self._zmq_context.socket(zmq.PUB)              #Zmq publisher
        socket_out.bind("tcp://*:%s" % port)
        
        self._print('ZMQ_publisher {0} loop start'.format(ident))
        while(self._thread_other_dict[ident]['run']):
            string = socket_in.recv()
            socket_out.send(string)
            self._count[3] += 1
        
        socket_in.close()
        socket_out.close()
        self._print('ZMQ_publisher {0} loop start'.format(ident))
        
    @in_thread
    def Plugin(self, funct, infinite = False,*argv, **kwargs):
        ##########################################################
        #
        #Plugin wrapper
        #The plugin function has to take a zmq subscriber socket (blocking socket) as first argument
        #It will be run in a new thread 
        #
        ##########################################################
        connection_out_number = len(self._thread_other_dict)                  #Define the number of the connection
        ident = '{0:2d}'.format(connection_out_number)
        self._thread_other_dict[ident] = {'thread' : threading.currentThread(), 'run' : True }     #Auto registration of the thread


        socket_in = self._zmq_context.socket(zmq.SUB)
        socket_in.connect('inproc://out')                           #Connect to bridge output
        socket_in.setsockopt(zmq.SUBSCRIBE, '')
        
        if(infinite):
            self._print('Plugin {0} loop start'.format(ident))
            while(self._thread_other_dict[ident]['run']):
                funct(socket_in, *argv, **kwargs)                           
            self._print('Plugin {0} loop stop'.format(ident))
        else:
            self._print('Plugin {0} one shot start'.format(ident))
            funct(socket_in, *argv, **kwargs)                           
            self._print('Plugin {0} one shot stop'.format(ident))
        socket_in.close()

    @in_thread
    def ZQM_reply(self, port):
        connection_out_number = len(self._thread_other_dict)                  #Define the number of the connection
        ident = '{0:02d}'.format(connection_out_number)
        self._thread_other_dict[ident] = {'thread' : threading.currentThread(), 'run' : True }     #Auto registration of the thread

        #Bind to external port
        socket = self._zmq_context.socket(zmq.REP)              #Zmq publisher
        socket.bind("tcp://*:%s" % port)
        
        self._print('ZMQ_reply {0} loop start'.format(ident))
        while(self._thread_other_dict[ident]['run']):
            string = socket.recv()
            logging.debug(string)
            target, messagedata = string.split(" ",1)
            cmd_dict = loads(messagedata)
            if(target in self._thread_mav_dict):
                mav = self._thread_mav_dict[target]
                #try:
                #logging_string = '{0} : Not yet implemented'.format(cmd_dict['cmd'])
                # if (topic == 'MAVLINK_CMD'):
                    # if (cmd_dict['cmd'] == 'RESET'):
                        # self._mav.reboot_autopilot()
                        # logging_string = 'Launch reboot_autopilot command'
                    # elif (cmd_dict['cmd'] == 'LOITER_MODE'):
                        # self._mav.set_mode_loiter()
                        # logging_string = 'Launch set_mode_loiter command'
                    # elif (cmd_dict['cmd'] == 'RTL_MODE'):
                        # self._mav.set_mode_rtl()
                        # logging_string = 'Launch set_mode_rtl command'
                    # elif (cmd_dict['cmd'] == 'MISSION_MODE'):
                        # self._mav.set_mode_auto()
                        # logging_string = 'Launch set_mode_auto command'
                    # elif(cmd_dict['cmd'] == 'WP_LIST_REQUEST'):
                        # self._mav.waypoint_request_list_send()
                        # logging_string = 'Launch waypoint_request_list_send command'
                    # elif(cmd_dict['cmd'] == 'WP_REQUEST'):
                        # self._mav.waypoint_request_send(cmd_dict['seq'])
                        # logging_string = 'Launch waypoint_request_send({0}) command'.format(cmd_dict['seq'])
                    # if(self._logging):
                        # logging.debug(logging_string)     
            else:
                result = 'Error : Target not valid'
            socket.send(result)
            self._count[4] += 1
        
        socket.close()
        self._print('ZMQ_reply {0} loop start'.format(ident))
        
    def _print(self, _string):
        print(_string)
        logging.info(_string)
        
    def server_forever(self):
        while(True):
            try:
                sleep(1)
                if (self._verbose):
                    print('[MAVLINK IN, ZQM OUT, ZQM IN] : {0}'.format(self._count))
            except(KeyboardInterrupt, SystemExit):
                string = 'Exit called !!\nRecv/Send: {0}'.format(self._count)
                self._print(string)
                exit(0)
                            
    def verbose(self, switch):
        if(switch == True or switch == False):
            self._verbose = switch

if(__name__ == "__main__"):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--mavlink", type=str, help="define MAVLINK input", default="COM8")
    parser.add_argument("--baud", type=int, help="set baud rate if COM connection", default=57600)
    parser.add_argument("--dialect", type=str, help="define MAVLINK dialect", default="pixhawk")
    parser.add_argument("--zmq_port_out", type=int, help="define ZMQ port to publish MAVLINK data", default=42017)
    #parser.add_argument("--zmq_in", type=int, help="define ZMQ port to suscribe to external commands", default=42018)
    parser.add_argument("--verbose", help="set verbose output", action="store_true")
    parser.add_argument("--logging", help="log DEBUG info in MAVLINK_plug.log file", action="store_true")
    parser.add_argument("--prefix", type=str, help="prefix for zmq message (will be followed by connection number)", default="M")
    args = parser.parse_args()
 
    my_plug = MAVLINK_Plug(args.prefix)
    
    if(args.verbose):
        my_plug.verbose(True)
    if(args.logging):
        logging.basicConfig(filename='MAVLINK_plug.log',level=logging.DEBUG,format='[%(levelname)s] %(asctime)s (%(threadName)-10s) %(message)s', filemode = 'w')
    my_plug.MAVLINK_connection(args.mavlink, baud=args.baud, dialect=args.dialect)
    my_plug.ZMQ_publisher(args.zmq_port_out)
    my_plug.server_forever()
                            