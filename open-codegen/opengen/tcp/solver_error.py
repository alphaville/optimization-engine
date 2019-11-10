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
            attribute_name = "__{}".format(k)
            setattr(self, attribute_name, v)

    @property
    def code(self):
        """
        Returns error codes:
        1000: Invalid request: Malformed or invalid JSON
        1600: Initial guess has incomplete dimensions
        1700: Wrong dimension of Lagrange multipliers
        2000: Problem solution failed (solver error)
        3003: Parameter vector has wrong length
        :return: Error code
        """
        return self.__dict__["__code"]

    @property
    def message(self):
        """
        Returns an appropriate error message matching the error code
        :return: Error message
        """
        return self.__dict__["__message"]
