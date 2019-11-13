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
            self.__response = SolverError(d)
        else:
            self.__response = SolverStatus(d)

    def is_ok(self):
        """Determines if response is OK."""
        return isinstance(self.__response, SolverStatus)

    def get(self):
        """
        Returns response, which is an instance of SolverStatus,
        if the call was successful, or an instance of SolverError
        otherwise. It is recommended that you use is_ok() to check
        whether the call has succeeded first
        """
        return self.__response

    def __getitem__(self, key):
        return getattr(self.__response, key)
