import casadi.casadi as cs
import numpy as np
from .constraint import Constraint
import opengen.functions as fn


class AffineSpace(Constraint):
    """An affine constraint

    A constraint of the form :math:`Ax = b`, where :math:`A` and :math:`b` are 
    a matrix and a vector of appropriate dimensions
    """

    def __init__(self, A, b):
        """Constructor for an affine space

        :return: new instance of AffineSpace
        """
        self.__A = A.flatten('C')
        self.__b = b

    @property
    def matrix_a(self):
        """Matrix A
        """
        return self.__A

    @property
    def vector_b(self):
        """Vector b        
        """
        return self.__b

    def distance_squared(self, u):
        """Squared distance to affine space

        Not implemented yet
        """
        raise NotImplementedError()

    def project(self, u):
        """Projection on affine space

        Not implemented yet
        """
        raise NotImplementedError()

    def is_convex(self):
        """Affine spaces are convex sets
        """
        return True

    def is_compact(self):
        """Affine spaces are not compact sets
        """
        return False
