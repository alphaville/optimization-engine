#![deny(missing_docs)]
//! Constraints and projections
//!
//! This module defines the trait [`Constraint`], which specifies an abstract
//! projection method, and a collection of simple sets, such as norm-balls,
//! finite sets, second-order cones and their Cartesian products.
//!
//!
//! [`Constraint`]: trait.Constraint.html

use crate::FunctionCallResult;

mod affine_space;
mod ball1;
mod ball2;
mod ballinf;
mod ballp;
mod cartesian_product;
mod epigraph_squared_norm;
mod finite;
mod halfspace;
mod hyperplane;
mod no_constraints;
mod rectangle;
mod simplex;
mod soc;
mod sphere2;
mod zero;

pub use affine_space::{AffineSpace, AffineSpaceError};
pub use ball1::Ball1;
pub use ball2::Ball2;
pub use ballinf::BallInf;
pub use ballp::BallP;
pub use cartesian_product::CartesianProduct;
pub use epigraph_squared_norm::EpigraphSquaredNorm;
pub use finite::FiniteSet;
pub use halfspace::Halfspace;
pub use hyperplane::Hyperplane;
pub use no_constraints::NoConstraints;
pub use rectangle::Rectangle;
pub use simplex::Simplex;
pub use soc::SecondOrderCone;
pub use sphere2::Sphere2;
pub use zero::Zero;

/// A set which can be used as a constraint
///
/// This trait defines an abstract function that allows to compute projections
/// on sets; this is implemented by a series of structures (see below for details)
pub trait Constraint<T = f64> {
    /// Projection onto the set, that is,
    ///
    /// $$
    /// \Pi_C(v) = \mathrm{argmin}_{z\in C}\Vert{}z-v{}\Vert
    /// $$
    ///
    /// ## Arguments
    ///
    /// - `x`: The given vector $x$ is updated with the projection on the set
    ///
    /// ## Errors
    ///
    /// Implementations should return `Err(...)` when the projection cannot be
    /// computed reliably because of a numerical or internal failure.
    ///
    /// ## Panics
    ///
    /// Implementations may still panic on clear API misuse, such as calling the
    /// projection with a slice of incompatible dimension.
    ///
    fn project(&self, x: &mut [T]) -> FunctionCallResult;

    /// Returns true if and only if the set is convex
    fn is_convex(&self) -> bool;
}

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
#[cfg(test)]
mod tests;
