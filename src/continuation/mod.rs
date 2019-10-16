//! *Deprecated* - use `alm` instead
//!
//! Continuation/Homotopy methods for parametric optimization problems
//!
//! Consider a parametric optimization problem of the general form
//!
//! ```txt
//! Minimize_u f(u; p)
//! subject to: u in U(p)
//! ```
//!
//! Suppose that for a particular value of `p`, this problem is ill
//! conditioned.
//!
//! It then makes sense to solve the problem for a sequence of values
//! `p_k`, which converge to the target value `p`, starting from an
//! initial value `p_0` and using each solution as a warm start for the
//! next optimization problem.
//!
//! The target value of `p` can be equal to infinity. In that case we
//! have the so-called "penalty method".
//!
//! The above parametric problem is accompanied by a function `c(u; p)`,
//! used as a termination criterion. In particular, when `c(u; p)`
//! drops below a desired tolerance, then the iterative procedure
//! is terminated.
//!

mod homotopy_cache;
mod homotopy_optimizer;
mod homotopy_problem;
mod homotopy_solver_status;

pub use homotopy_cache::HomotopyCache;
pub use homotopy_optimizer::HomotopyOptimizer;
pub use homotopy_problem::HomotopyProblem;
pub use homotopy_solver_status::HomotopySolverStatus;

/// Continuation mode for free parameters
#[derive(Debug, Clone, Copy)]
pub enum ContinuationMode {
    /// Arithmetic progression
    ///
    /// A free parameter with initial value `x: f64` and target value
    /// `y: f64` is updated by adding a constant parameter `s: f64`.
    /// Parameter `y` is allowed to be `std::f64::INFINITY`. If `y` is
    /// finite, the continuation will terminate in a finite number of
    /// steps.
    ///
    Arithmetic(f64),
    /// Convex combination
    ///
    /// Can only be applied when the initial and final values, `x` and
    /// `y`, are finite. The update is a convex combination of the current
    /// value and `y`
    Convex(f64),
    /// Is typically used when `y` is plus or minus infinity or zero.
    /// Then, the parameter is updated by multiplying by a given factor.
    Geometric(f64),
}

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
// #[cfg(test)]
//mod tests;
