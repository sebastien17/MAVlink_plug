============
MAVlink_plug
============

MAVlink_plug is a python script that can be used to publish MAVlink message on a JSON format with the ZMQ library.

It uses the pymavlink library to connect to the MAVLink endpoint.

The MAVLink plug will define a ZQM publisher to send each MAVlink message,  as they arrive, in a JSON format(dictionary) preceded by his MAVlink message type.

Some commands can be sent to the MAVlink_plug using a ZQM pub-sub channel.