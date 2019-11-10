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
    def type(self):
        """
        Returns value at key __type
        :return: "Error"
        """
        return self.__dict__["__type"]

    @property
    def code(self):
        """
        Returns value at key __code
        :return: Error code
        """
        return self.__dict__["__code"]

    @property
    def message(self):
        """
        Returns value at key __message
        :return: Error message matching the code
        """
        return self.__dict__["__message"]
