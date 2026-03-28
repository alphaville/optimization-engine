//! Forward-Backward Splitting algorithm
//!
//! ## About
//!
//! This module offers an implementation of the forward-backward splitting algorithm
//!
//!
//!
//! ## Example: Forward-Backward Splitting
//!
//! ```
//! use optimization_engine::{*, fbs::*};
//! use optimization_engine::constraints::Ball2;
//! use std::num::NonZeroUsize;
//!
//! fn my_cost(u: &[f64], cost: &mut f64) -> Result<(), SolverError> {
//!     *cost = u[0] * u[0] + 2. * u[1] * u[1] + u[0] - u[1] + 3.0;
//!     Ok(())
//! }
//!
//! fn my_gradient(u: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
//!     grad[0] = u[0] + u[1] + 1.0;
//!     grad[1] = u[0] + 2. * u[1] - 1.0;
//!     Ok(())
//! }
//!
//! let radius = 0.2;
//! let box_constraints = Ball2::new(None, radius);
//! let problem = Problem::new(&box_constraints, my_gradient, my_cost);
//! let gamma = 0.3;
//! let tolerance = 1e-6;
//!
//! let mut fbs_cache = FBSCache::new(NonZeroUsize::new(2).unwrap(), gamma, tolerance);
//! let mut u = [0.0; 2];
//! let mut optimizer = FBSOptimizer::new(problem, &mut fbs_cache);
//!
//! let status = optimizer.solve(&mut u).unwrap();
//!
//! assert!(status.has_converged());
//! ```

mod fbs_cache;
mod fbs_engine;
mod fbs_optimizer;

pub use fbs_cache::FBSCache;
pub use fbs_optimizer::FBSOptimizer;

/* --------------------------------------------------------------------------------------------- */
/*          TESTS                                                                                */
/* --------------------------------------------------------------------------------------------- */

#[cfg(test)]
mod tests;
