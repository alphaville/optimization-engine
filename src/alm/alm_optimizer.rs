#![allow(dead_code)]

use crate::{
    alm::*,
    constraints,
    core::{panoc::PANOCOptimizer, Optimizer, Problem, SolverStatus},
    matrix_operations, SolverError,
};

const DEFAULT_MAX_OUTER_ITERATIONS: usize = 50;
const DEFAULT_MAX_INNER_ITERATIONS: usize = 5000;
const DEFAULT_EPSILON_TOLERANCE: f64 = 1e-6;
const DEFAULT_DELTA_TOLERANCE: f64 = 1e-4;
const DEFAULT_PENALTY_UPDATE_FACTOR: f64 = 5.0;
const DEFAULT_EPSILON_UPDATE_FACTOR: f64 = 0.1;
const DEFAULT_INFEAS_SUFFICIENT_DECREASE_FACTOR: f64 = 0.1;
const DEFAULT_INITIAL_TOLERANCE: f64 = 0.1;

pub struct AlmOptimizer<
    'life,
    MappingAlm,
    MappingPm,
    ParametricGradientType,
    ConstraintsType,
    AlmSetC,
    LagrangeSetY,
    ParametricCostType,
> where
    MappingAlm: Fn(&[f64], &mut [f64]) -> Result<(), SolverError>,
    MappingPm: Fn(&[f64], &mut [f64]) -> Result<(), SolverError>,
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
        MappingAlm,
        MappingPm,
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
        MappingAlm,
        MappingPm,
        ParametricGradientType,
        ConstraintsType,
        AlmSetC,
        LagrangeSetY,
        ParametricCostType,
    >
    AlmOptimizer<
        'life,
        MappingAlm,
        MappingPm,
        ParametricGradientType,
        ConstraintsType,
        AlmSetC,
        LagrangeSetY,
        ParametricCostType,
    >
where
    MappingAlm: Fn(&[f64], &mut [f64]) -> Result<(), SolverError>,
    MappingPm: Fn(&[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> Result<(), SolverError>,
    ConstraintsType: constraints::Constraint,
    AlmSetC: constraints::Constraint,
    LagrangeSetY: constraints::Constraint,
{
    pub fn new(
        alm_cache: &'life mut AlmCache,
        alm_problem: AlmProblem<
            MappingAlm,
            MappingPm,
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
            penalty_update_factor: DEFAULT_PENALTY_UPDATE_FACTOR,
            epsilon_update_factor: DEFAULT_EPSILON_UPDATE_FACTOR,
            sufficient_decrease_coeff: DEFAULT_INFEAS_SUFFICIENT_DECREASE_FACTOR,
            epsilon_inner_initial: DEFAULT_INITIAL_TOLERANCE,
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
        if let Some(xi_in_cache) = &mut cache.xi {
            xi_in_cache[1..].copy_from_slice(y);
        }
    }

    pub fn set_penalty_init(&mut self, c0: f64) {
        if let Some(xi_in_cache) = &mut self.alm_cache.xi {
            xi_in_cache[0] = c0;
        }
    }

    //TODO: We're lacking a few setter methods

    fn compute_alm_infeasibility(&mut self) -> Result<(), SolverError> {
        let alm_cache = &mut self.alm_cache; // ALM cache
        if let (Some(y_plus), Some(xi)) = (&alm_cache.y_plus, &alm_cache.xi) {
            let norm_diff_squared: f64 = matrix_operations::norm2_squared_diff(&y_plus, &xi[1..]);
            alm_cache.delta_y_norm = norm_diff_squared.sqrt();
        }
        Ok(())
    }

    /// Computes PM infeasibility, that is, ||F2(u)||
    fn compute_pm_infeasibility(&mut self, u: &[f64]) -> Result<(), SolverError> {
        let problem = &self.alm_problem; // ALM problem
        let cache = &mut self.alm_cache; // ALM cache

        // If there is an F2 mapping: cache.w_pm <-- F2
        // TODO: we are interested in the norm of F2(u)
        if let Some(f2) = &problem.mapping_f2 {
            if let Some(w_pm_vec) = cache.w_pm.as_mut() {
                f2(u, w_pm_vec)?
            }
        }
        Ok(())
    }

    /// Updates the Lagrange multipliers using
    ///
    /// `y_plus <-- y + c*[F1(u_plus) - Proj_C(F1(u_plus) + y/c)]`
    ///
    fn update_lagrange_multipliers(&mut self, u: &[f64]) -> Result<(), SolverError> {
        let problem = &self.alm_problem; // ALM problem
        let cache = &mut self.alm_cache; // ALM cache

        // y_plus <-- y + c*[F1(u_plus) - Proj_C(F1(u_plus) + y/c)]
        // This is implemented as follows:
        //
        // #1. w_alm_aux := F1(u), where u = solution of inner problem
        // #2. y_plus := w_alm_aux + y/c
        // #3. y_plus := Proj_C(y_plus)
        // #4. y_plus := y + c(w_alm_aux - y_plus)

        // Before we start: this should not be executed if n1 = 0
        if problem.n1 == 0 {
            return Ok(());
        }

        // Step #1
        if let (Some(f1), Some(w_alm_aux)) = (&problem.mapping_f1, &mut cache.w_alm_aux) {
            (f1)(u, w_alm_aux)?; // w_alm_aux := F1(u)

            // Step #2
            if let (Some(y_plus), Some(xi)) = (&mut cache.y_plus, &mut cache.xi) {
                let y = &xi[1..];
                let c = xi[0];
                // y_plus := w_alm_aux + y/c
                y_plus
                    .iter_mut()
                    .zip(y.iter())
                    .zip(w_alm_aux.iter())
                    .for_each(|((y_plus_i, y_i), w_alm_aux_i)| *y_plus_i = w_alm_aux_i + y_i / c);

                // Step #3
                if let Some(alm_set_c) = &problem.alm_set_c {
                    // y_plus := Proj_C(y_plus)
                    alm_set_c.project(y_plus);
                }

                // Step #4
                y_plus
                    .iter_mut()
                    .zip(y.iter())
                    .zip(w_alm_aux.iter())
                    .for_each(|((y_plus_i, y_i), w_alm_aux_i)| {
                        // y_plus := y + c(w_alm_aux - y_plus)
                        *y_plus_i = y_i + c * (w_alm_aux_i - *y_plus_i)
                    });
            }
        }

        Ok(())
    }

    /// Project y on set Y
    fn project_on_set_y(&mut self) {
        let problem = &self.alm_problem;
        if let Some(y_set) = &problem.alm_set_y {
            // NOTE: as_mut() converts from &mut Option<T> to Option<&mut T>
            // * cache.y is                Option<Vec<f64>>
            // * cache.y.as_mut is         Option<&mut Vec<f64>>
            // *  which can be treated as  Option<&mut [f64]>
            // * y_vec is                  &mut [f64]
            if let Some(xi_vec) = self.alm_cache.xi.as_mut() {
                y_set.project(&mut xi_vec[1..]);
            }
        }
    }

    /// Solve inner problem
    ///
    /// ## Arguments
    ///
    /// - `u`: (on entry) current iterate, `u^nu`, (on exit) next iterate,
    ///   `u^{nu+1}` which is an epsilon-approximate solution of the inner problem
    /// - `xi`: vector `xi = (c, y)`
    ///
    /// ## Returns
    ///
    /// Returns an instance of `Result<SolverStatus, SolverError>`, where `SolverStatus`
    /// is the solver status of the inner problem and `SolverError` is a potential
    /// error in solving the inner problem.
    ///
    ///
    fn solve_inner_problem(&mut self, u: &mut [f64]) -> Result<SolverStatus, SolverError> {
        let alm_problem = &self.alm_problem; // Problem
        let alm_cache = &mut self.alm_cache; // ALM cache

        // `xi` is either the cached `xi` if one exists, or an reference to an
        // empty vector, otherwise. We do that becaues the user has the option
        // to not use any ALM/PM constraints; in that case, `alm_cache.xi` is
        // `None`
        let xi_empty = Vec::new();
        let xi = if let Some(xi_cached) = &alm_cache.xi {
            &xi_cached
        } else {
            &xi_empty
        };
        // Construct psi and psi_grad (as functions of `u` alone); it is
        // psi(u) = psi(u; xi) and psi_grad(u) = phi_grad(u; xi)
        // psi: R^nu --> R
        let psi = |u: &[f64], psi_val: &mut f64| -> Result<(), SolverError> {
            (alm_problem.parametric_cost)(u, &xi, psi_val)
        };
        // psi_grad: R^nu --> R^nu
        let psi_grad = |u: &[f64], psi_grad: &mut [f64]| -> Result<(), SolverError> {
            (alm_problem.parametric_gradient)(u, &xi, psi_grad)
        };
        // define the inner problem
        let inner_problem = Problem::new(&self.alm_problem.constraints, psi_grad, psi);
        // TODO: tolerance decrease until target tolerance is reached
        let mut inner_solver = PANOCOptimizer::new(inner_problem, &mut alm_cache.panoc_cache);
        // this method returns the result of .solve:
        inner_solver.solve(u)
    }

    /// Step of ALM algorithm
    fn step(&mut self, u: &mut [f64]) -> Result<bool, SolverError> {
        // Project y on Y
        self.project_on_set_y();
        // If the inner problem fails miserably, the failure should be propagated
        // upstream (using `?`). If the inner problem has not converged, that is fine,
        // we should keep solving.
        self.solve_inner_problem(u)
            .map(|_status: SolverStatus| {})?;
        // Update Lagrange multipliers:
        // y_plus <-- y + c*[F1(u_plus) - Proj_C(F1(u_plus) + y/c)]
        self.update_lagrange_multipliers(u)?;
        // Compute infeasibilities
        self.compute_pm_infeasibility(u)?;
        self.compute_alm_infeasibility()?;
        // Check exit criterion

        Ok(true)
    }

    /// Solve the specified ALM problem
    ///
    ///
    pub fn solve(&mut self, u: &mut [f64]) -> Result<(), SolverError> {
        // TODO: implement loop - check output of .step()
        let _step_result = self.step(u);
        Ok(())
    }
}
