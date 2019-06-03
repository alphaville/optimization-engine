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
        if radius <= 0:
            raise Exception("The radius must be a positive number")

        if center is not None and not isinstance(center, list):
            raise Exception("center is neither None nor a list")

        self.__center = None if center is None else [float(i) for i in center]
        self.__radius = float(radius)

    @property
    def center(self):
        """Returns the center of the ball"""
        return self.__center

    @property
    def radius(self):
        """Returns the radius of the ball"""
        return self.__radius

