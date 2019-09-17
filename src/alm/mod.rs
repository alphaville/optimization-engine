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

pub type MappingType = fn(&[f64], &mut [f64]) -> Result<(), crate::SolverError>;
pub type JacobianMappingType = fn(&[f64], &[f64], &mut [f64]) -> Result<(), crate::SolverError>;

pub const NO_MAPPING: Option<MappingType> = None::<MappingType>;
pub const NO_JACOBIAN_MAPPING: Option<JacobianMappingType> = None::<JacobianMappingType>;
pub const NO_SET: Option<crate::constraints::NoConstraints> =
    None::<crate::constraints::NoConstraints>;

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
#[cfg(test)]
mod tests;
