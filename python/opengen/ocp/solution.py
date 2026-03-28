"""High-level solution objects returned by OCP-generated optimizers."""

from numbers import Real

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
        self.penalty = getattr(raw, "penalty", None)
        self.num_outer_iterations = getattr(raw, "num_outer_iterations", None)
        self.num_inner_iterations = getattr(raw, "num_inner_iterations", None)
        self.last_problem_norm_fpr = getattr(raw, "last_problem_norm_fpr", None)
        self.f1_infeasibility = getattr(raw, "f1_infeasibility", None)
        self.f2_norm = getattr(raw, "f2_norm", None)
        self.lagrange_multipliers = getattr(raw, "lagrange_multipliers", None)

    def __repr__(self):
        """Return a readable multi-line summary of the solution."""
        def fmt(value):
            if isinstance(value, bool):
                return str(value)
            if isinstance(value, Real):
                numeric = float(value)
                if numeric == 0.0:
                    return "0.0"
                text = f"{numeric:.4g}"
                return "0.0" if text == "-0" else text
            if isinstance(value, list):
                return "[" + ", ".join(fmt(item) for item in value) + "]"
            if isinstance(value, tuple):
                return "(" + ", ".join(fmt(item) for item in value) + ")"
            return str(value)

        return "\n".join([
            "OCP Solution:",
            f"  Exit status.......... {self.exit_status}",
            f"  Cost................. {fmt(self.cost)}",
            f"  Solve time [ms]...... {fmt(self.solve_time_ms)}",
            f"  Penalty.............. {fmt(self.penalty)}",
            f"  Outer iterations..... {fmt(self.num_outer_iterations)}",
            f"  Inner iterations..... {fmt(self.num_inner_iterations)}",
            f"  FPR.................. {fmt(self.last_problem_norm_fpr)}",
            f"  ALM infeasibility.... {fmt(self.f1_infeasibility)}",
            f"  PM infeasibility..... {fmt(self.f2_norm)}",
            f"  Decision variables... {fmt(self.solution)}",
            f"  Inputs............... {fmt(self.inputs)}",
            f"  States............... {fmt(self.states)}",
            f"  Lagrange multipliers. {fmt(self.lagrange_multipliers)}",
        ])

    __str__ = __repr__
