import mavlinkplug.QuadCopter
from time import sleep

if(__name__ == '__main__'):
    quad = mavlinkplug.QuadCopter.QuadCopter(zmq_in = 'tcp://127.0.0.1:45063', zmq_out = 'tcp://127.0.0.1:45064')
    quad.start()
    sleep(10)
    quad.stop()