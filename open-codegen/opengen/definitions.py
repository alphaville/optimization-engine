import os
import sys


def open_codegen_root_dir():
    return os.path.dirname(sys.modules['opengen'].__file__)


def open_root_dir():
    return os.path.abspath(os.path.join(open_codegen_root_dir(), "..", ".."))


def templates_dir():
    return os.path.abspath(os.path.join(open_codegen_root_dir(), "..", "templates"))


def original_icasadi_dir():
    return os.path.abspath(os.path.join(open_codegen_root_dir(), "..", "icasadi"))
