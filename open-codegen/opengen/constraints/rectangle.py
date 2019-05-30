class Rectangle:
    """A Rectangle (Box) constraint"""

    def __init__(self, xmin, xmax):
        """Construct a new instance of Rectangle

        Args:
            xmin: minimum bounds (can be None)
            xmax: maximum bounds (can be None)

        Raises:
            Exception: if both xmin and xmax is None
            Exception: if xmin/xmax is not None and not a list (wrong type)
            Exception: if xmin and xmax have incompatible lengths
            Exception: if xmin(i) > xmax(i) for some i (empty set)

        Returns:
             A new instance of Rectangle
        """
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
        self.__xmin = None if xmin is None else [float(i) for i in xmin]
        self.__xmax = None if xmax is None else [float(i) for i in xmax]

    @property
    def xmin(self):
        """Minimum bound"""
        return self.__xmin

    @property
    def xmax(self):
        """Maximum bound"""
        return self.__xmax

