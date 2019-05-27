import os
import sys
import pkg_resources


def templates_dir():
    return pkg_resources.resource_filename('opengen', 'templates/')


def original_icasadi_dir():
    return pkg_resources.resource_filename('opengen', 'icasadi/')
