from .solver_status import SolverStatus
from .solver_error import SolverError

class SolverResponse:
    """Stores a solver response of type SolverStatus or SolverError."""
    def __init__(self, d):
        """Constructs instance of <code>SolverResponse</code>

        Args:
            d: dictionary containing either status or error attributes

        Returns:
            New instance of <code>SolverResponse</code>
        """
        if 'Error' in d.values():
            self.response = SolverError(d)
        else:
            self.response = SolverStatus(d)

    def is_ok(self):
        """Determines if response is OK."""
        return isinstance(self.response, SolverStatus)

    def is_err(self):
        """Determines if response is an error."""
        return isinstance(self.response, SolverError)

    def get(self):
        """Returns response."""
        return self.response

    def get_dict(self):
        return self.response.__dict__
