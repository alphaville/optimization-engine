class Rectangle:
    """A Rectangle (Box) constraint"""

    def __init__(self, xmin, xmax):
        # (None, None) is not allowed
        if xmin is None and xmax is None:
            raise Exception("At least one of xmin and xmax must be not None")

        # If one is not None, it must be a list
        if xmin is not None and not isinstance(xmin, list):
            raise Exception("xmin is neither None nor a list")

        if xmax is not None and not isinstance(xmax, list):
            raise Exception("xmax is neither None nor a list")

        # If BOTH are not None, they must be compatible
        if xmin is not None and xmax is not None:
            if len(xmin) != len(xmax):
                raise Exception("xmin and xmax must have equal lengths")
            for (xmin_element, xmax_element) in zip(xmin, xmax):
                if xmin_element > xmax_element:
                    raise Exception("xmin must be <= xmax")

        # Store xmin and xmax in attributes
        self.__xmin = xmin
        self.__xmax = xmax

    @property
    def xmin(self):
        return self.__xmin

    @property
    def xmax(self):
        return self.__xmax

    @xmin.setter
    def xmin(self, xmin):
        self.__xmin = xmin

    @xmax.setter
    def xmax(self, xmax):
        self.__xmax = xmax
