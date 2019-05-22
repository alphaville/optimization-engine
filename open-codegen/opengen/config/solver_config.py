class SolverConfiguration:

    def __init__(self):
        self._tolerance = 1e-4
        self._lbfgs_memory = 10
        self._max_iterations = 500
        self._max_time_millis = 20