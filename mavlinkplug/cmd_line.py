#!/usr/bin/env python
# -*- coding: utf-8 -*-

def mavlinkplug_cmd_line():
    doc = '''mavlinkplug
    This tools has to be plugged to a MAVLINK device.
    It can redirect MAVLINK message to:
        - TCP port (to connect Ground Station for example)
        - ZMQ port
        - Log file (JSON or binary format)
        

    Usage:
      mavlinkplug <mavlink> [--baud=<baud_rate>] [--dialect=<dialect>] [--tcp=<tcp_port>] [--file=<file>] [--bin=<file>] [--zmq=<zmq_port>] [--verbose]
      
    Options:
      --baud=<baud_rate>    Define baud rate for a MAVLINK serial connection
      --dialect=<dialect>   Define dialect for a MAVLINK connection
      --tcp=<tcp_port>      Active and define the tcp output on <tcp_port>
      --file=<file>         Active and define the file output in <file> (data in JSON format)
      --bin=<file>          Active and define the file output in <file>  (binary)
      --zmq=<zmq_port>      Active and define the zmq output on <tcp_port>
      --verbose             Show modules information
      --help                Show this screen
    '''
    
    import docopt
    import mavlinkplug
    
    arguments = docopt.docopt(doc)
    print('MAVLINK Connection : {0}'.format(arguments['<mavlink>']))
    mavlink_connection_args = {}
    if (arguments['--baud'] != None):
        mavlink_connection_args['baud'] = arguments['--baud']
        print('Baud : {0}'.format(arguments['--baud']))
    if (arguments['--dialect'] != None):
        mavlink_connection_args['dialect'] = arguments['--dialect']   
        print('Dialect : {0}'.format(arguments['--dialect']))
    #Initialise plug
    my_plug = mavlinkplug.Plug()
    if(arguments['--verbose'] == True):
        my_plug.verbose(True)
    #Connection independant module    
    if(arguments['--file'] != None):
         my_plug.FILE_out(arguments['--file'])
    if(arguments['--bin'] != None):
         my_plug.BIN_out(arguments['--bin'])
    if(arguments['--zmq'] != None):
        my_plug.ZMQ_out(arguments['--zmq'])
    #Active MAVLINK connection
    mav_con01 = my_plug.MAVLINK_in(arguments['<mavlink>'], **mavlink_connection_args)
    #Connection dependant module
    if(arguments['--tcp'] != None):
        my_plug.TCP_in_out(mav_con01,'',int(arguments['--tcp']))
    my_plug.server_forever()

if __name__ == "__main__":
    mavlinkplug_cmd_line()