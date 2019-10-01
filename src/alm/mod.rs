#![deny(missing_docs)]
//! Augmented Lagrangian and Penalty Methods
//!
//! A module that contains structures and implementations that allow
//! to formulate parametric constrained nonconvex optimization problems
//! in the form specified in [`AlmProblem`]
//!
//! Such problems can then be solved using [`AlmOptimizer`], which combines the
//! augmented Lagrangian and the penalty method.
//!
//! The user needs to create an [`AlmCache`] object, which can then be passed to
//! different instances of `AlmOptimizer`. An `AlmCache` allocates the necessary
//! memory that the optimizer needs.
//!
//! Upon completion of its execution, `AlmOptimizer` returns information about
//! the iterative procedure, such as the solution time, number of iterations,
//! measures of accuracy and more, in the form of an [`AlmOptimizerStatus`]
//!
//! When using `AlmOptimizer`,  the user is expected to provide a modified cost
//! function, `psi` (see [`AlmOptimizer`] for details). This should not be a problem
//! for users that use Optimization Engine via its Python or MATLAB interfaces.
//! Should the user need to use Optimization Engine in Rust, she can construct
//! function `psi` using [`AlmFactory`]
//!
//! [`AlmProblem`]: struct.AlmProblem.html
//! [`AlmOptimizer`]: struct.AlmOptimizer.html
//! [`AlmCache`]: struct.AlmCache.html
//! [`AlmOptimizerStatus`]: struct.AlmOptimizerStatus.html
//! [`AlmFactory`]: struct.AlmFactory.html
//!
mod alm_cache;
mod alm_factory;
mod alm_optimizer;
mod alm_optimizer_status;
mod alm_problem;

pub use alm_cache::AlmCache;
pub use alm_factory::AlmFactory;
pub use alm_optimizer::AlmOptimizer;
pub use alm_optimizer_status::AlmOptimizerStatus;
pub use alm_problem::AlmProblem;

/// Type of mappings $F_1(u)$ and $F_2(u)$
///
/// Mappings $F_1$ and $F_2$ are computed by functions with signature
///
/// ```ignore
/// fn mapping_f(&[f64], &mut [f64]) -> Result<(), crate::SolverError>
/// ```
pub type MappingType = fn(&[f64], &mut [f64]) -> Result<(), crate::SolverError>;

/// Type of the Jacobian of mappings $F_1$ and $F_2$
///
/// These are mappings $(u, d) \mapsto JF_1(u)^\top d$, for given vectors $u\in\mathbb{R}$
/// and $d\in\mathbb{R}^{n_1}$ (similarly for $F_2$)
pub type JacobianMappingType = fn(&[f64], &[f64], &mut [f64]) -> Result<(), crate::SolverError>;

/// No mapping $F_1(u)$ or $F_2(u)$ is specified
pub const NO_MAPPING: Option<MappingType> = None::<MappingType>;

/// No Jacobian mapping is specified for $F_1$ and $F_2$
pub const NO_JACOBIAN_MAPPING: Option<JacobianMappingType> = None::<JacobianMappingType>;

/// No set is specified (when specifying a set is optional)
pub const NO_SET: Option<crate::constraints::NoConstraints> =
    None::<crate::constraints::NoConstraints>;

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
#[cfg(test)]
mod tests;
