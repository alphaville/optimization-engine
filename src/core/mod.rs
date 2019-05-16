//! Optimisation algorithms
//!
//!
use crate::constraints;
use std::time;

pub mod fbs;
pub mod panoc;
pub mod problem;
pub mod solver_status;

/// Solver status
///
/// This structure contais information about the solver status. Instances of
/// `SolverStatus` are returned by optimizers.
///
#[derive(Debug)]
pub struct SolverStatus {
    /// whether the algorithm has converged
    converged: bool,
    /// number of iterations for convergence
    num_iter: usize,
    /// time it took to solve
    solve_time: time::Duration,
    /// norm of the fixed-point residual (FPR)
    fpr_norm: f64,
    /// cost value at the candidate solution
    cost_value: f64,
}

/// A general optimizer
pub trait Optimizer {
    /// solves a given problem and updates the initial estimate `u` with the solution
    ///
    /// Returns the solver status
    ///
    fn solve(&mut self, u: &mut [f64]) -> SolverStatus;
}

/// Engine supporting an algorithm
///
/// An engine is responsible for the allocation of memory for an algorithm,
/// especially memory that is reuasble is multiple instances of the same
/// algorithm (as in model predictive control).
///
/// It defines what the algorithm does at every step (see `step`) and whether
/// the specified termination criterion is satisfied
///
pub trait AlgorithmEngine {
    /// Take a step of the algorithm and return `true` only if the iterations should continue
    fn step(&mut self, u: &mut [f64]) -> bool;

    fn init(&mut self, u: &mut [f64]);
}

/// Definition of an optimisation problem
///
/// The definition of an optimisation problem involves:
/// - the gradient of the cost function
/// - the cost function
/// - the set of constraints, which is described by implementations of
///   [Constraint](../../panoc_rs/constraints/trait.Constraint.html)
pub struct Problem<GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    CostType: Fn(&[f64], &mut f64) -> i32,
    ConstraintType: constraints::Constraint,
{
    /// constraints
    constraints: ConstraintType,
    /// gradient of the cost
    gradf: GradientType,
    /// cost function
    cost: CostType,
}
