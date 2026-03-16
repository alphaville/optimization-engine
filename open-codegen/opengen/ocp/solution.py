class OcpSolution:
    """High-level solution object returned by GeneratedOptimizer.solve."""

    def __init__(self, raw, inputs, states):
        self.raw = raw
        self.inputs = inputs
        self.states = states
        self.solution = getattr(raw, "solution", [])
        self.cost = getattr(raw, "cost", None)
        self.exit_status = getattr(raw, "exit_status", None)
        self.solve_time_ms = getattr(raw, "solve_time_ms", None)
        self.lagrange_multipliers = getattr(raw, "lagrange_multipliers", None)
