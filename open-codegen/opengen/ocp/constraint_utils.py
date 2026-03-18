"""Helpers for simplifying products of OCP constraint sets."""

from opengen.constraints.cartesian import CartesianProduct
from opengen.constraints.no_constraints import NoConstraints
from opengen.constraints.rectangle import Rectangle
from opengen.constraints.zero import Zero


def segment_dimensions(segments):
    """Compute segment dimensions from Cartesian-product end indices."""
    dimensions = []
    previous = -1
    for segment_end in segments:
        dimensions.append(segment_end - previous)
        previous = segment_end
    return dimensions


def rectangle_bounds(constraint, dimension):
    """Return explicit box bounds when a set admits a rectangle representation."""
    if isinstance(constraint, NoConstraints):
        return [float("-inf")] * dimension, [float("inf")] * dimension

    if isinstance(constraint, Zero):
        return [0.0] * dimension, [0.0] * dimension

    if isinstance(constraint, Rectangle):
        if constraint.dimension() != dimension:
            raise ValueError("constraint dimension does not match its segment length")
        xmin = constraint.xmin if constraint.xmin is not None else [float("-inf")] * dimension
        xmax = constraint.xmax if constraint.xmax is not None else [float("inf")] * dimension
        return list(xmin), list(xmax)

    return None


def make_constraint_product(segments, constraints):
    """Build the most specific set representing a block product of constraints."""
    if not constraints:
        return NoConstraints()

    if len(constraints) == 1:
        return constraints[0]

    if all(isinstance(constraint, NoConstraints) for constraint in constraints):
        return NoConstraints()

    if all(isinstance(constraint, Zero) for constraint in constraints):
        return Zero()

    dimensions = segment_dimensions(segments)
    rectangle_data = [
        rectangle_bounds(constraint, dimension)
        for constraint, dimension in zip(constraints, dimensions)
    ]

    if all(bounds is not None for bounds in rectangle_data):
        xmin = []
        xmax = []
        for current_xmin, current_xmax in rectangle_data:
            xmin.extend(current_xmin)
            xmax.extend(current_xmax)
        return Rectangle(xmin, xmax)

    return CartesianProduct(segments, constraints)
