#!/usr/bin/env python
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

from mavlinkplug.Exception import MAVlinkPlugException
from struct import pack

_PACK_FORMAT = '!B'




class _MetaClassList(type):
    def __getattr__(cls, name):
        if(name in cls._clist):
            return cls._clist.index(name)
        elif(name.endswith('_P')):
            return pack(_PACK_FORMAT, cls.__getattr__(name[:-2]))
        else:
            raise MAVlinkPlugException('MSG_PLUG_TYPE {0} not existing'.format(name))
    def __contains__(cls, item):
        return True if(item in cls._clist) else False

#Message Plug Type List
class MSG_PLUG_TYPE(object):
    __metaclass__ = _MetaClassList
    _clist = [
            'MAV_MSG',
            'MAV_COMMAND',
            'KILL',
            ]

#Message Plug Endpoint List
class MSG_PLUG_END_POINT(object):
    MSG_PLUG_DEST_TYPE_ALL = 255
    MSG_PLUG_DEST_TYPE_ALL_P = pack(_PACK_FORMAT,255)