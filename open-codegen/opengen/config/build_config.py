from opengen.definitions import *


class BuildConfiguration:

    def __init__(self):
        self.target_system = 'default'
        self.build_mode = 'release'
        self.build_path = default_build_dir()
        self.casadi_path = None
