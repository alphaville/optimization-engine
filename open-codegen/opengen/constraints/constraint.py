class Constraint:

    def distance_squared(self, u):
        raise NotImplementedError("Method `distance_squared` is not implemented")

    def project(self, u):
        raise NotImplementedError("Method `project` is not implemented")

    def is_convex(self):
        return False

    def is_compact(self):
        return False
