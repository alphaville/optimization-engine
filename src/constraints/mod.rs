//! Constraints and projections

mod ball2;
mod no_constraints;
mod rectangle;

pub use ball2::Ball2;
pub use no_constraints::NoConstraints;
pub use rectangle::Rectangle;

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

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
#[cfg(test)]
mod tests;
