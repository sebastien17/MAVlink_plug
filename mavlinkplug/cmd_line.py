#!/usr/bin/env python
# -*- coding: utf-8 -*-

def mavlinkplug_cmd_line():
    import argparse
    import mavlinkplug
    parser = argparse.ArgumentParser()
    parser.add_argument("--mavlink", type=str, help="define MAVLINK input", default="COM8")
    parser.add_argument("--baud", type=int, help="set baud rate if COM connection", default=57600)
    parser.add_argument("--dialect", type=str, help="define MAVLINK dialect", default="pixhawk")
    parser.add_argument("--zmq_port_out", type=int, help="define ZMQ port to publish MAVLINK data", default=42017)
    #parser.add_argument("--zmq_in", type=int, help="define ZMQ port to suscribe to external commands", default=42018)
    parser.add_argument("--verbose", help="set verbose output", action="store_true")
    parser.add_argument("--logging", help="log DEBUG info in MAVLINK_plug.log file", action="store_true")
    parser.add_argument("--prefix", type=str, help="prefix for zmq message (will be followed by connection number)", default="")
    args = parser.parse_args()
    my_plug = mavlinkplug.Plug(args.prefix)

    if(args.verbose):
        my_plug.verbose(True)
    if(args.logging):
        logging.basicConfig(filename='MAVLINK_plug.log',level=logging.DEBUG,format='[%(levelname)s] %(asctime)s (%(threadName)-10s) %(message)s', filemode = 'w')
    my_plug.MAVLINK_connection(args.mavlink, baud=args.baud, dialect=args.dialect)
    my_plug.ZMQ_publisher(args.zmq_port_out)
    my_plug.server_forever()
 