class Constraint:

    def distance_squared(self, u):
        raise NotImplementedError("Method `distance_squared` is not implemented")

    def project(self, u):
        raise NotImplementedError("Method `project` is not implemented")

    def is_convex(self):
        return False

    def is_compact(self):
        return False

    def sup_level_set(self, x):
        raise NotImplementedError("Method `sup_level_set` is not implemented")

    def get_scaled_constraint(self, scaling_factor):
        raise NotImplementedError("Method `get_scaled_constraint` is not implemented")
