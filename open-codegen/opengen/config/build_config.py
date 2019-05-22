from opengen.definitions import *


class BuildConfiguration:

    def __init__(self):
        self.target_system = 'default'
        self.build_mode = 'release'
        self.cost_function_name = 'phi'
        self.grad_cost_function_name = 'grad_phi'
        self.constraint_penalty_function = 'constraints_penalty'
