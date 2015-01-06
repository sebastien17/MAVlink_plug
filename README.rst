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
