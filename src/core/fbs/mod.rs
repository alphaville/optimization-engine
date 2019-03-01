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
//! use optimization_engine::core::{Problem,Optimizer};
//! use optimization_engine::core::fbs::{FBSCache,FBSEngine,FBSOptimizer};
//! use optimization_engine::constraints::Ball2;
//! use std::num::NonZeroUsize;
//!
//! fn my_cost(u: &[f64], cost: &mut f64) -> i32 {
//!     *cost = u[0] * u[0] + 2. * u[1] * u[1] + u[0] - u[1] + 3.0;
//!     0
//! }
//!
//! fn my_gradient(u: &[f64], grad: &mut [f64]) -> i32 {
//!     grad[0] = u[0] + u[1] + 1.0;
//!     grad[1] = u[0] + 2. * u[1] - 1.0;
//!     0
//! }
//!
//! fn main() {
//!     let radius = 0.2;
//!     let box_constraints = Ball2::new_at_origin_with_radius(radius);
//!     let problem = Problem::new(box_constraints, my_gradient, my_cost);
//!     let gamma = 0.3;
//!     let tolerance = 1e-6;
//!
//!     let mut fbs_cache = FBSCache::new(NonZeroUsize::new(2).unwrap(), gamma, tolerance);
//!     let mut fbs_step = FBSEngine::new(problem, &mut fbs_cache);
//!     let mut u = [0.0; 2];
//!     let mut optimizer = FBSOptimizer::new(&mut fbs_step);
//!
//!     let status = optimizer.solve(&mut u);
//!
//!     assert!(status.has_converged());
//! }
//! ```
mod fbs_cache;
mod fbs_engine;
mod fbs_optimizer;

use super::Problem;
use crate::constraints;

/// Cache for the forward-backward splitting (FBS), or projected gradient, algorithm
///
/// This struct allocates memory needed for the FBS algorithm
pub struct FBSCache {
    work_gradient_u: Vec<f64>,
    work_u_previous: Vec<f64>,
    gamma: f64,
    tolerance: f64,
    norm_fpr: f64,
}

/// The FBE Engine defines the steps of the FBE algorithm and the termination criterion
///
pub struct FBSEngine<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    CostType: Fn(&[f64], &mut f64) -> i32,
    ConstraintType: constraints::Constraint,
{
    problem: Problem<GradientType, ConstraintType, CostType>,
    cache: &'a mut FBSCache,
}

/// Optimiser using forward-backward splitting iterations (projected gradient)
///
/// Note that an `FBSOptimizer` holds a reference to an instance of `FBSEngine`
/// which needs to be created externally. A mutable reference to that `FBSEgnine`
/// is provided to the optimizer.
///
/// The `FBSEngine` is supposed to be updated whenever you need to solve
/// a different optimization problem.
///
///
pub struct FBSOptimizer<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    CostType: Fn(&[f64], &mut f64) -> i32,
    ConstraintType: constraints::Constraint,
{
    fbs_engine: &'a mut FBSEngine<'a, GradientType, ConstraintType, CostType>,
    max_iter: usize,
}

/* --------------------------------------------------------------------------------------------- */
/*          TESTS                                                                                */
/* --------------------------------------------------------------------------------------------- */

#[cfg(test)]
mod tests;
