from opengen.definitions import *


class BuildConfiguration:

    def __init__(self, build_dir=None):
        self._target_system = 'default'
        self._build_mode = 'release'
        self._cost_function_name = 'phi'
        self._grad_cost_function_name = 'grad_phi'
        self._constraint_penalty_function = 'constraints_penalty'
        self._rebuild = False
        self._build_dir = build_dir
        self._open_version = '*'

    # ---------- GETTERS ---------------------------------------------

    def rebuild(self):
        return self._rebuild

    def cost_function_name(self):
        return self._cost_function_name

    def grad_function_name(self):
        return self._grad_cost_function_name

    def constraint_penalty_function_name(self):
        return self._constraint_penalty_function

    def target_system(self):
        return self._target_system

    def build_mode(self):
        return self._build_mode

    def build_dir(self):
        return self._build_dir

    def open_version(self):
        return self._open_version

    # ---------- SETTERS ---------------------------------------------

    def with_rebuild(self, do_rebuild):
        self._rebuild = do_rebuild
        return self

    def with_cost_function_name(self, cost_function_name):
        self._cost_function_name = cost_function_name
        return self

    def with_grad_function_name(self, grad_function_name):
        self._grad_cost_function_name = grad_function_name
        return self

    def with_penalty_constraints_function_name(self, constraint_penalty_function_name):
        self._constraint_penalty_function = constraint_penalty_function_name
        return self

    def with_target_system(self, target_system):
        self._target_system = target_system
        return self

    def with_build_mode(self, build_mode):
        self._build_mode = build_mode
        return self

    def with_build_directory(self, build_dir):
        self._build_dir = build_dir
        return self

    def with_open_version(self, open_version):
        self._open_version = open_version
        return self
