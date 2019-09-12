mod alm_cache;
mod alm_optimizer;
mod alm_problem;

pub use alm_cache::AlmCache;
pub use alm_optimizer::AlmOptimizer;
pub use alm_problem::AlmProblem;

pub type MappingType = fn(&[f64], &mut [f64]) -> Result<(), crate::SolverError>;

pub const NO_MAPPING: Option<MappingType> = None::<MappingType>;
pub const NO_SET: Option<crate::constraints::NoConstraints> =
    None::<crate::constraints::NoConstraints>;

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
#[cfg(test)]
mod tests;
