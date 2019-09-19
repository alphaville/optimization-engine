#![deny(missing_docs)]
//! Augmented Lagrangian and Penalty Methods
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

/// Type of mappings `F1` and `F2`
///
/// Mappings `F1` and `F2` are computed by functions with signature
///
/// ```ignore
/// fn mapping_f(&[f64], &mut [f64]) -> Result<(), crate::SolverError>
/// ```
pub type MappingType = fn(&[f64], &mut [f64]) -> Result<(), crate::SolverError>;

/// Type of the Jacobian of mappings `F1` and `F2`
///
/// These are mappings $(u, d) \mapsto JF_1(u)^\top d$, for given vectors $u\in\mathbb{R}$
/// and $d\in\mathbb{R}^{n_1}$ (similarly for `F2`)
pub type JacobianMappingType = fn(&[f64], &[f64], &mut [f64]) -> Result<(), crate::SolverError>;

/// No mapping `F1` or `F2` is specified
pub const NO_MAPPING: Option<MappingType> = None::<MappingType>;

/// No Jacobian mapping is specified for `F1` and `F2`
pub const NO_JACOBIAN_MAPPING: Option<JacobianMappingType> = None::<JacobianMappingType>;

/// No set is specified (when specifying a set is optional)
pub const NO_SET: Option<crate::constraints::NoConstraints> =
    None::<crate::constraints::NoConstraints>;

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
#[cfg(test)]
mod tests;
