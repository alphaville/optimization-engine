class SolverStatus:
    """Class for storing the status of the solver."""
    def __init__(self, status):
        """Constructs instance of <code>SolverStatus</code>

        Args:
            status: dictionary containing solver status attributes

        Returns:
            New instance of <code>SolverStatus</code>
        """

        for k, v in status.items():
            setattr(self, k, v)
