from ._version import get_versions

#Logging Parameters
import logging

__version__ = get_versions()['version']
del get_versions

# Managing Mavlink dialect

if(not '_MAVLINKPLUG_DIALECT' in globals() ):
    _MAVLINKPLUG_DIALECT = "ardupilotmega" # Default dialect value

def set_mavlink_dialect(dialect = None):
    if(dialect != None):
        global _MAVLINKPLUG_DIALECT
        _MAVLINKPLUG_DIALECT = dialect