//! Constraints and projections

mod ball2;
mod no_constraints;
mod rectangle;

/// A set which can be used as a constraint
///
/// This trait defines an abstract function that allows to compute projections
/// on the set
pub trait Constraint {
    /// Projection onto the set
    ///
    /// The given vector `x` is updated with the projection on the set
    fn project(&self, x: &mut [f64]);
}

/// The whole space, no constraints
pub struct NoConstraints {}

/// A Eucledian ball
pub struct Ball2 {
    centre: Option<Vec<f64>>,
    radius: f64,
}

///
/// A rectangle
///
/// A set of the form `{x : xmin <= x <= xmax}`, where `<=` is meant in the
/// element-wise sense and either of `xmin` and `xmax` can be equal to infinity.
pub struct Rectangle {
    xmin: Option<Vec<f64>>,
    xmax: Option<Vec<f64>>,
}

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
#[cfg(test)]
mod tests;
