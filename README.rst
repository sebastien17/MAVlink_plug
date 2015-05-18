============
mavlinkplug
============

mavlinkplug is a python library for relaying, logging and processing MAVLink message.

Features
--------

* Simple
* Highly Modular
* Command-line tools :

mavlinkplug
 This tools has to be plugged to a MAVLINK device.
 It can redirect MAVLINK message to:
     - TCP port (to connect Ground Station for example)
     - ZMQ port
     - Log file (JSON or binary format)
     
 
 Usage:
.. code-block:: bash
 
 mavlinkplug <mavlink> [--baud=<baud_rate>] [--dialect=<dialect>] [--tcp=<tcp_port>] [--file=<file>] [--bin=<file>] [--zmq=<zmq_port>] [--verbose]

Example of Use
--------------
.. code-block:: python

 import mavlinkplug
 
 my_plug = mavlinkplug.Plug()
 my_plug.verbose(True)
 my_plug.FILE_out('output.txt')
 my_plug.BIN_out('output.bin')
 my_plug.ZMQ_out('tcp://localhost:33333')
 mav_con01 = my_plug.MAVLINK_in('com4', baud=115200, dialect=pixhawk)
 my_plug.TCP_in_out(mav_con01,'',17501))
 my_plug.server_forever()

Dependencies
------------

* pymavlink   : to connect to the MAVLink endpoint.
* pyserial    : for pymavlink dependencies
* pyzmq       : for internal messaging and external zmq publishment.
* docopt      : for command-line parsing.

Install
-------

* Clone the Github repository
* Install the library:
.. code-block:: bash

 python setup.py install
