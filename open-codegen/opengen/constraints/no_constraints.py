from .constraint import Constraint


class NoConstraints(Constraint):
    """
    This is the entire :math:`{\\rm I\!R}^n`

    """

    def __init__(self):
        pass

    def distance_squared(self, u):
        return 0.0

    def project(self, u):
        return u

    def is_convex(self):
        return True

    def is_compact(self):
        return False
