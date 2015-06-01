from ._version import get_versions
from .Plug import *
#from .Module import *

__version__ = get_versions()['version']
del get_versions
