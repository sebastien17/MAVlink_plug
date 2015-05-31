#	-*- coding: utf-8 -*-
import mavlinkplug, time
from mavlinkplug.QuadCopter import QuadCopter
from mavlinkplug.Hil import Hil

if(__name__ == '__main__'):
    my_plug = mavlinkplug.Plug()
    #mav_con01 = my_plug.MAVLINK_in('COM3',dialect='pixhawk')
    hil_test = Hil(QuadCopter, my_plug)
    #hil_test.FL_initialize()
    hil_test.start()
    time.sleep(10)
