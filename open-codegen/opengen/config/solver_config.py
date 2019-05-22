class SolverConfiguration:

    def __init__(self):
        self._tolerance = 1e-4
        self._lbfgs_memory = 10
        self._max_iterations = 500
        self._max_time_millis = 20

    def tolerance(self):
        return self._tolerance

    def lbfgs_memory(self):
        return self._lbfgs_memory

    def max_iterations(self):
        return self._max_iterations

    def max_time_millis(self):
        return self._max_time_millis

    def with_tolerance(self, tolerance):
        self._tolerance = tolerance
        return self

    def with_lfbgs_memory(self, lbfgs_memory):
        self._lbfgs_memory = lbfgs_memory
        return self
