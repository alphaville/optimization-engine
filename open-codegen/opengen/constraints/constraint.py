class Constraint:

    def distance_squared(self, u):
        raise NotImplementedError(
            "Method `distance_squared` is not implemented")

    def project(self, u):
        """
        Project a given point onto the set

        :param u: given point

        :return: projection of ``u`` onto this set
        """
        raise NotImplementedError("Method `project` is not implemented")

    def is_convex(self):
        """
        Whether the set is convex 
        """
        return False

    def is_compact(self):
        """
        Whether the set is compact
        """
        return False
