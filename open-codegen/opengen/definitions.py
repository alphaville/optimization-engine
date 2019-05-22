import os
import sys

def open_root_dir():
    return os.path.dirname(sys.modules['__main__'].__file__)
