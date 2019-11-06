class SolverError:
    """Class for storing solver status in the event of an error."""
    def __init__(self, error):
        """Constructs instance of <code>SolverError</code>

        Args:
            error: dictionary containing error attributes

        Returns:
            New instance of <code>SolverError</code>
        """

        for k, v in error.items():
            setattr(self, k, v)
