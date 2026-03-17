"""High-level solution objects returned by OCP-generated optimizers."""

class OcpSolution:
    """High-level solution returned by :meth:`GeneratedOptimizer.solve`.

    The object wraps the raw low-level solver status and adds OCP-oriented
    views such as stage-wise inputs and the reconstructed state trajectory.
    """

    def __init__(self, raw, inputs, states):
        """Construct an OCP solution object.

        :param raw: raw low-level solver status
        :param inputs: stage-wise input sequence
        :param states: state trajectory
        """
        self.raw = raw
        self.inputs = inputs
        self.states = states
        self.solution = getattr(raw, "solution", [])
        self.cost = getattr(raw, "cost", None)
        self.exit_status = getattr(raw, "exit_status", None)
        self.solve_time_ms = getattr(raw, "solve_time_ms", None)
        self.lagrange_multipliers = getattr(raw, "lagrange_multipliers", None)

    def __repr__(self):
        """Return a readable multi-line summary of the solution."""
        return "\n".join([
            "OCP Solution:",
            f"  Exit status.......... {self.exit_status}",
            f"  Cost................. {self.cost}",
            f"  Solve time [ms]...... {self.solve_time_ms}",
            f"  Decision variables... {self.solution}",
            f"  Inputs............... {self.inputs}",
            f"  States............... {self.states}",
            f"  Lagrange multipliers. {self.lagrange_multipliers}",
        ])

    __str__ = __repr__
