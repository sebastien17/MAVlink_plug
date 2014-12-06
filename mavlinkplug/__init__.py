from ._version import get_versions
from .plug import *

__version__ = get_versions()['version']
del get_versions
