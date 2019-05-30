class Ball2:
    """A Euclidean ball constraint

    A constraint of the form ||u-u0|| <= r, where u0 is the center
    of the ball and r is its radius

    """

    def __init__(self, center, radius):
        """Constructor for a Euclidean ball constraint

        Args:
            center: center of the ball; if this is equal to Null, the
            ball is centered at the origin

            radius: radius of the ball

        Returns:
            New instance of Ball2 with given center and radius
        """
        assert(radius > 0)
        self.__center = center
        self.__radius = radius

    @property
    def center(self):
        """Returns the center of the ball"""
        return self.__center

    @property
    def radius(self):
        """Returns the radius of the ball"""
        return self.__radius

    @center.setter
    def center(self, center):
        self.__center = center

    @radius.setter
    def radius(self, radius):
        assert(radius > 0)
        self.__radius = radius
