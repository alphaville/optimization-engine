from .solver_status import SolverStatus
from .solver_error import SolverError


class SolverResponse:
    """Stores a solver response of type SolverStatus or SolverError."""

    def __init__(self, d):
        """Constructs instance of :class:`~opengen.tcp.solver_response.SolverResponse`

        This constructor is not to be used by the end-user

        :param d: dictionary containing either status or error attributes
        :type d: dictionary

        :return: New instance of :class:`~opengen.tcp.solver_response.SolverResponse`
        """
        if 'Error' in d.values():
            self.__response = SolverError(d)
        else:
            self.__response = SolverStatus(d)

    def is_ok(self):
        """Determines if response is OK.

        This method should always be called first when obtaining a server response.
        """
        return isinstance(self.__response, SolverStatus)

    def get(self):
        """
        Returns response, which is an instance of SolverStatus,
        if the call was successful, or an instance of SolverError
        otherwise. It is recommended that you use 
        :class:`~opengen.tcp.solver_response.SolverResponse.is_ok` to check
        whether the call has succeeded first
        """
        return self.__response

    def __getitem__(self, key):
        return getattr(self.__response, key)

    def __repr__(self):
        if self.is_ok():
            status = self.get()
            return (
                "SolverResponse(ok=True, "
                f"exit_status={status.exit_status!r}, "
                f"num_outer_iterations={status.num_outer_iterations}, "
                f"num_inner_iterations={status.num_inner_iterations})"
            )
        error = self.get()
        return (
            "SolverResponse(ok=False, "
            f"code={error.code}, "
            f"message={error.message!r})"
        )
