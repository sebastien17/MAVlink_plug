from ._version import get_versions

#Logging Parameters
import logging
logging.basicConfig(filename='example.log',level=logging.DEBUG)

__version__ = get_versions()['version']
del get_versions
