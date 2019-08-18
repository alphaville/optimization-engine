//! Constraints and projections

mod ball2;
mod cartesian_product;
mod finite;
mod no_constraints;
mod rectangle;
mod soc;

pub use ball2::Ball2;
pub use cartesian_product::CartesianProduct;
pub use finite::FiniteSet;
pub use no_constraints::NoConstraints;
pub use rectangle::Rectangle;
pub use soc::SecondOrderCone;

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
