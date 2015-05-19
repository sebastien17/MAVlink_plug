#	-*- coding: utf-8 -*-

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#	This file is part of MAVlinkplug.

#	MAVlinkplug is free software: you can redistribute it and/or modify
#	it under the terms of the GNU General Public License as published by
#	the Free Software Foundation, either version 3 of the License, or
#	(at your option) any later version.

#	MAVlinkplug is distributed in the hope that it will be useful,
#	but WITHOUT ANY WARRANTY; without even the implied warranty of
#	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#	GNU General Public License for more details.

#	You should have received a copy of the GNU General Public License
#	along with MAVlinkplug.  If not, see <http://www.gnu.org/licenses/>.
#	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import mavlinkplug, pyfdm
from pymavlink.dialects.v10 import pixhawk as mavlink
import zmq, time

#Method added to Plug class
def __function_to_add(self, *argv, **kwargs):
    ident = '{0:02d}'.format(len(self._input_list))
    h = mavlinkplug.hil.hil(self._zmq_context, self._zmq_bridge_out, ident, *argv, **kwargs)
    h.run()
    self._input_list.append(h)
    return h

#Add hil method to Plug class (defined by __function_to_add)
mavlinkplug.Plug.plugin_register('hil', __function_to_add)
    
class hil(mavlinkplug.ModBase):
    '''
    Module for HIL simulation
    Translate information from MAVLINK Connection to  FL and back
    '''
    def __init__(self, zmq_context, from_plug_zmq_sock, ident, mav_connection, to_FL_zmq_sock_port, from_FL_zmq_sock_port,FL_instance):
        super(hil, self).__init__()
        self._zmq_context = zmq_context
        self._from_plug_zmq_sock = from_plug_zmq_sock
        self._ident = ident
        self._mav = mav_connection
        self._to_FL_zmq_sock_port = to_FL_zmq_sock_port
        self._from_FL_zmq_sock_port = from_FL_zmq_sock_port
        self._out_msg = 0
        self._in_msg = 0
        #ZMQ sockets initialization
        # self._from_plug_zmq_socket = self._zmq_context.socket(zmq.SUB)
        # self._from_plug_zmq_socket.connect(self._from_plug_zmq_sock)                                                #Connect to bridge output
        # self._from_plug_zmq_socket.setsockopt(zmq.SUBSCRIBE, mavlinkplug.ZMQ_MESSAGE_JSON + self._mav.ident())      #Filter on encrypted message with mav ident
        # self._to_FL_zmq_socket = self._zmq_context.socket(zmq.PUB)
        # self._to_FL_zmq_socket.bind(self._to_FL_zmq_sock_port)                                                       
        # self._from_FL_zmq_socket = self._zmq_context.socket(zmq.SUB)
        # self._from_FL_zmq_socket.bind(self._from_FL_zmq_sock_port)                                          
        # self._from_FL_zmq_socket.setsockopt(zmq.SUBSCRIBE, '')                                                  #No filters
        self._FL_instance = FL_instance
        self._hardware_ready = False
        self._FL_ready = False
        print('HIL instance {0} initialized'.format(self._ident))
    
    def _get_mode_flag(self, flag):
        if (self._mav.mav_handle().base_mode & flag) == 0:
            return False
        else:
            return True

    def _set_mode_flag(self, flag, enable):
        t_start = time.time()
        if self._get_mode_flag(flag) == enable:
            return
        while not self._get_mode_flag(flag) == enable:
            self._mav.mav_handle().set_mode_flag(flag, enable)
            while self._mav.mav_handle().port.inWaiting() > 0:
                m = self._mav.mav_handle().recv_msg()
            time.sleep(0.1)
            if time.time()  - t_start > 5: raise IOError('Failed to set mode flag, check port')
    def _set_hil_and_arm(self):
        t_start = time.time()
        if (self._get_mode_flag(mavlink.MAV_MODE_FLAG_HIL_ENABLED) and
           self._get_mode_flag(mavlink.MAV_MODE_FLAG_SAFETY_ARMED)):
            return
        while (not
               (self._get_mode_flag(mavlink.MAV_MODE_FLAG_HIL_ENABLED)
               and 
               self._get_mode_flag(mavlink.MAV_MODE_FLAG_SAFETY_ARMED))):
            self._mav.mav_handle().mav.command_long_send(self._mav.mav_handle().target_system,
                                self._mav.mav_handle().target_component,
                                mavlink.MAV_CMD_DO_SET_MODE, 4,
                                mavlink.MAV_MODE_FLAG_SAFETY_ARMED |
                                mavlink.MAV_MODE_FLAG_HIL_ENABLED,
                                0, 0, 0, 0, 0, 0)
            while self._mav.mav_handle().port.inWaiting() > 0:
                m = self._mav.mav_handle().recv_msg()
            time.sleep(0.1)
            if time.time()  - t_start > 5: raise IOError('Failed to '\
                    + 'transition to HIL mode and arm, check port and firmware')

    # def wait_for_no_msg(self, msg, period, timeout, callback=None):
        # done = False
        # t_start = time.time()
        # t_last = time.time()
        # while not done:
            # if callback is not None: callback()
            # while self.master.port.inWaiting() > 0:
                # m = self.master.recv_msg()
                # if m is None: continue
                # if m.get_type() == msg:
                    # t_last = time.time()
            # if time.time() - t_last > period:
                # done = True
            # elif time.time() - t_start > timeout:
                # done = False
                # break
            # time.sleep(0.001)

        # return done
 
    # def wait_for_msg(self, msg, timeout, callback=None):
        # done = False
        # t_start = time.time()
        # while not done:
            # if callback is not None: callback()
            # while self.master.port.inWaiting() > 0:
                # m = self.master.recv_msg()
                # if m is None: continue
                # if m.get_type() == msg:
                    # done = True
                    # break
            # if time.time() - t_start > timeout:
                # done = False
                # break
            # time.sleep(0.1)

        # return done

    # def reboot_autopilot(self):

        # reboot_successful = False
        # while not reboot_successful:

            # # Request reboot until no heartbeat received
                # # The callback option cannot be used for this, because it
                # # runs at the same speed as the message receive which is
                # # unecessary.
            # shutdown = False
            # while not shutdown:
                # self.set_hil_and_arm()
                # #self.set_mode_flag(mavlink.MAV_MODE_FLAG_HIL_ENABLED, True)
                # print 'Sending reboot to autopilot'
                # self.master.reboot_autopilot()
                # # wait for heartbeat timeout, continue looping if not received
                # shutdown = self.wait_for_no_msg(msg='HEARTBEAT', period=2, timeout=10)
            # print 'Autopilot heartbeat lost (rebooting)'
            # # Try to read heartbeat three times before restarting shutdown
            # for i in range(3):
                # print 'Attempt %d to read autopilot heartbeat.' % (i+1)
                # # Reset serial comm
                # self.master.reset()
                # reboot_successful = self.wait_for_msg('HEARTBEAT', timeout=100)
                # if reboot_successful:
                    # #delay sending data to avoid boot problem on px4
                    # time.sleep(1)
                    # self.set_hil_and_arm()
                    # #self.set_mode_flag(mavlink.MAV_MODE_FLAG_HIL_ENABLED, True)
                    # break
    
    def hardware_initialize(self):
        while(self._mav.mav_handle() == None):
            time.sleep(1)
        self._mav.mavlink_command('SET_HIL_ARM')
        #TODO : add check
        time.sleep(2)
        self._mav.mavlink_command('RESET')
        #TODO : add check
        self._hardware_ready = True
        return self._hardware_ready
    def FL_initialize(self):
        self._FL_ready = True
        return self._FL_ready
    def _mod(self):
        #Need 2 threads
        self._mav_2_FL()
        self._FL_2_mav()

    @mavlinkplug.in_thread(True)
    def _mav_2_FL(self):
        pass
    @mavlinkplug.in_thread(True)
    def _FL_2_mav(self):
        pass
    def info(self):
        pass
        #return {'ident' :  self._ident, 'address': self._address, 'msg_stats': {'out_msg': self._out_msg,'in_msg': self._in_msg, 'connection activated': self._connection_activated}}



if __name__ == "__main__":
    pass