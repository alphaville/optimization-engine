#![allow(dead_code)]

const DEFAULT_MAX_OUTER_ITERATIONS: usize = 50;
const DEFAULT_MAX_INNER_ITERATIONS: usize = 5000;
const DEFAULT_EPSILON_TOLERANCE: f64 = 1e-6;
const DEFAULT_DELTA_TOLERANCE: f64 = 1e-4;
const DEFAULT_PENALTY_UPDATE_FACTOR

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
    /// ALM cache (borrowed)
    alm_cache: &'life mut AlmCache,
    /// ALM problem definition (oracle)
    alm_problem: AlmProblem<
        ParametricMappingAlm,
        ParametricMappingPm,
        ParametricGradientType,
        ConstraintsType,
        AlmSetC,
        LagrangeSetY,
        ParametricCostType,
    >,
    /// Maximum number of outer iterations
    max_outer_iterations: usize,
    /// Maximum number of inner iterations
    max_inner_iterations: usize,
    /// Maximum duration
    max_duration: Option<std::time::Duration>,
    /// epsilon for inner AKKT condition
    epsilon_tolerance: f64,
    /// delta for outer AKKT condition
    delta_tolerance: f64,
    /// At every outer iteration, c is multiplied by this scalar
    penalty_update_factor: f64,
    /// The epsilon-tolerance is multiplied by this factor until
    /// it reaches its target value
    epsilon_update_factor: f64,
    /// If current_infeasibility <= sufficient_decrease_coeff * previous_infeasibility,
    /// then the penalty parameter is kept constant
    sufficient_decrease_coeff: f64,
    // Initial tolerance (for the inner problem)
    epsilon_inner_initial: f64,
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
            max_outer_iterations: DEFAULT_MAX_OUTER_ITERATIONS,
            max_inner_iterations: DEFAULT_MAX_INNER_ITERATIONS,
            max_duration: None,
            epsilon_tolerance: DEFAULT_EPSILON_TOLERANCE,
            delta_tolerance: DEFAULT_DELTA_TOLERANCE,
            penalty_update_factor: 10.0,
            epsilon_update_factor: 0.1,
            sufficient_decrease_coeff: 10.0,
            epsilon_inner_initial: 0.1,
        }
    }

    pub fn with_max_outer_iterations(mut self, max_outer_iterations: usize) -> Self {
        self.max_outer_iterations = max_outer_iterations;
        self
    }

    pub fn with_max_inner_iterations(mut self, max_inner_iterations: usize) -> Self {
        self.max_inner_iterations = max_inner_iterations;
        self
    }

    pub fn with_max_duration(mut self, max_duration: std::time::Duration) -> Self {
        self.max_duration = Some(max_duration);
        self
    }

    pub fn with_delta_tolerance(mut self, delta_tolerance: f64) -> Self {
        self.delta_tolerance = delta_tolerance;
        self
    }

    pub fn set_lagrange_multipliers_init(&mut self, y: &[f64]) {
        let cache = &mut self.alm_cache;
        if let Some(y_in_cache) = &mut cache.y {
            y_in_cache.copy_from_slice(y);
        }
    }

    fn project_on_set_y(&'life mut self) {
        let cache = &mut self.alm_cache;
        let problem = &self.alm_problem;
        if let Some(y_set) = &problem.alm_set_y {
            // NOTE: as_mut() converts from &mut Option<T> to Option<&mut T>
            // * cache.y is                Option<Vec<f64>>
            // * cache.y.as_mut is         Option<&mut Vec<f64>>
            // *  which can be treated as  Option<&mut [f64]>
            // * y_vec is                  &mut [f64]
            if let Some(y_vec) = cache.y.as_mut() {
                y_set.project(y_vec);
            }
        }
    }

    fn compute_pm_infeasibility(
        &'life mut self,
        u: &[f64],
        params: &[f64],
    ) -> Result<(), SolverError> {
        let problem = &self.alm_problem;
        let cache = &mut self.alm_cache;
        // If there is an F2 mapping, apply it to (u, params)
        if let Some(f2) = &problem.mapping_f2 {
            if let Some(w_pm_vec) = cache.w_pm.as_mut() {
                f2(u, params, w_pm_vec)?
            }
        }
        Ok(())
    }

    pub fn solve(&'life mut self, _u: &mut [f64], _q: &[f64]) {
        self.project_on_set_y();
    }
}
