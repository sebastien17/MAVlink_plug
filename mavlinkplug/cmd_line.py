#!/usr/bin/env python
# -*- coding: utf-8 -*-

def mavlinkplug_cmd_line():
    doc = '''mavlinkplug
    This tools has to be plugged to a MAVLINK device.
    It can redirect MAVLINK message to:
        - TCP port (to connect Ground Station for example)
        - ZMQ port
        - Log file
        

    Usage:
      mavlinkplug <mavlink> [--baud=<baud_rate>] [--dialect=<dialect>] [--tcp=<tcp_port>] [--file=<file>] [--zmq=<zmq_port>] [--verbose]
      
    Options:
      --baud=<baud_rate>    Define baud rate for a MAVLINK serial connection
      --dialect=<dialect>   Define dialect for a MAVLINK connection
      --tcp=<tcp_port>      Active and define the tcp output on <tcp_port>
      --file=<file>         Active and define the file output in <file>
      --zmq=<zmq_port>      Active and define the zmq output on <tcp_port>
      --verbose             Show modules information
      -h --help             Show this screen.
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
    my_plug = mavlinkplug.Plug()
    mav_con01 = my_plug.MAVLINK_in(arguments['<mavlink>'], **mavlink_connection_args)
    if(arguments['--tcp'] != None):
        my_plug.TCP_in_out(mav_con01,'',int(arguments['--tcp']))
    if(arguments['--file'] != None):
         my_plug.FILE_out(arguments['--file'])
    if(arguments['--zmq'] != None):
        my_plug.ZMQ_out(arguments['--zmq'])
    if(arguments['--verbose'] == True):
        my_plug.verbose(True)
    my_plug.server_forever()
