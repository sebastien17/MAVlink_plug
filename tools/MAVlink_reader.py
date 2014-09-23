import sys
import zmq

port = "42017"

# Socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.SUB)

print "Collecting updates from MAVlink_plug..."
socket.connect ("tcp://localhost:%s" % port)


# Subscribe to HEARBEAT topic
topicfilter = "HEARTBEAT"
socket.setsockopt(zmq.SUBSCRIBE, topicfilter)

# Process 5 updates
total_value = 0
while True:
    string = socket.recv()                      #Blocking
    topic, messagedata = string.split(" ",1)
    print topic, messagedata