use crate::{alm::*, constraints, SolverError};

pub struct AlmOptimizer<
    'life,
    ParametricMappingAlm,
    ParametricMappingPm,
    ParametricGradientType,
    ConstraintsType,
    AlmSetC,
    LagrangeSetY,
    ParametricCostType,
> where
    ParametricMappingAlm: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricMappingPm: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> Result<(), SolverError>,
    ConstraintsType: constraints::Constraint,
    AlmSetC: constraints::Constraint,
    LagrangeSetY: constraints::Constraint,
{
    alm_cache: &'life mut AlmCache,
    alm_problem: AlmProblem<
        ParametricMappingAlm,
        ParametricMappingPm,
        ParametricGradientType,
        ConstraintsType,
        AlmSetC,
        LagrangeSetY,
        ParametricCostType,
    >,
    max_outer_iterations: usize,
    /// Maximum number of inner iterations
    max_inner_iterations: usize,
    /// Maximum duration
    max_duration: Option<std::time::Duration>,
    epsilon_tolerance: f64,
    delta_tolerance: f64,
    penalty_update_factor: f64,
    epsilon_update_factor: f64,
    sufficient_decrease_coeff: f64,
    epsilon_initial: f64,
}

impl<
        'life,
        ParametricMappingAlm,
        ParametricMappingPm,
        ParametricGradientType,
        ConstraintsType,
        AlmSetC,
        LagrangeSetY,
        ParametricCostType,
    >
    AlmOptimizer<
        'life,
        ParametricMappingAlm,
        ParametricMappingPm,
        ParametricGradientType,
        ConstraintsType,
        AlmSetC,
        LagrangeSetY,
        ParametricCostType,
    >
where
    ParametricMappingAlm: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricMappingPm: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> Result<(), SolverError>,
    ConstraintsType: constraints::Constraint,
    AlmSetC: constraints::Constraint,
    LagrangeSetY: constraints::Constraint,
{
    pub fn new(
        alm_cache: &'life mut AlmCache,
        alm_problem: AlmProblem<
            ParametricMappingAlm,
            ParametricMappingPm,
            ParametricGradientType,
            ConstraintsType,
            AlmSetC,
            LagrangeSetY,
            ParametricCostType,
        >,
    ) -> Self {
        AlmOptimizer {
            alm_cache,
            alm_problem,
            max_outer_iterations: 100,
            max_inner_iterations: 10000,
            max_duration: None,
            epsilon_tolerance: 1e-6,
            delta_tolerance: 1e-6,
            penalty_update_factor: 10.0,
            epsilon_update_factor: 0.1,
            sufficient_decrease_coeff: 10.0,
            epsilon_initial: 0.1,
        }
    }
}
