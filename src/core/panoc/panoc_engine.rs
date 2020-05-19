use crate::{
    constraints,
    core::{panoc::PANOCCache, AlgorithmEngine, Problem},
    matrix_operations, FunctionCallResult, SolverError,
};

/// gamma = GAMMA_L_COEFF/L
const GAMMA_L_COEFF: f64 = 0.95;

//const SIGMA_COEFF: f64 = 0.49;

/// Delta in the estimation of the initial Lipschitz constant
const DELTA_LIPSCHITZ: f64 = 1e-12;

/// Epsilon in the estimation of the initial Lipschitz constant
const EPSILON_LIPSCHITZ: f64 = 1e-6;

/// Safety parameter used to check a strict inequality in the update of the Lipschitz constant
const LIPSCHITZ_UPDATE_EPSILON: f64 = 1e-6;

/// Maximum iterations of updating the Lipschitz constant
const MAX_LIPSCHITZ_UPDATE_ITERATIONS: usize = 10;

/// Maximum possible Lipschitz constant
const MAX_LIPSCHITZ_CONSTANT: f64 = 1e9;

/// Maximum number of linesearch iterations
const MAX_LINESEARCH_ITERATIONS: u32 = 10;

/// Engine for PANOC algorithm
pub struct PANOCEngine<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> FunctionCallResult,
    CostType: Fn(&[f64], &mut f64) -> FunctionCallResult,
    ConstraintType: constraints::Constraint,
{
    problem: Problem<'a, GradientType, ConstraintType, CostType>,
    pub(crate) cache: &'a mut PANOCCache,
}

impl<'a, GradientType, ConstraintType, CostType>
    PANOCEngine<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> FunctionCallResult,
    CostType: Fn(&[f64], &mut f64) -> FunctionCallResult,
    ConstraintType: constraints::Constraint,
{
    /// Construct a new Engine for PANOC
    ///
    /// The engine owns a problem specification borrows a mutable reference
    /// to a `PANOCCache` object which is created externally once
    ///
    /// ## Arguments
    ///
    /// - `problem` problem specification (instance of [Problem](../struct.Problem.html))
    /// - `cache` an instance of [PANOCCache](struct.PANOCCache.html)
    ///
    ///
    pub fn new(
        problem: Problem<'a, GradientType, ConstraintType, CostType>,
        cache: &'a mut PANOCCache,
    ) -> PANOCEngine<'a, GradientType, ConstraintType, CostType> {
        PANOCEngine { problem, cache }
    }

    /// Estimate the local Lipschitz constant at `u`
    fn estimate_loc_lip(&mut self, u: &mut [f64]) -> FunctionCallResult {
        let mut lipest = crate::lipschitz_estimator::LipschitzEstimator::new(
            u,
            &self.problem.gradf,
            &mut self.cache.gradient_u,
        )
        .with_delta(DELTA_LIPSCHITZ)
        .with_epsilon(EPSILON_LIPSCHITZ);
        self.cache.lipschitz_constant = lipest.estimate_local_lipschitz()?;

        Ok(())
    }

    /// Computes the FPR and its norm
    fn compute_fpr(&mut self, u_current: &[f64]) {
        // compute the FPR:
        // fpr ← u - u_half_step
        let cache = &mut self.cache;
        cache
            .gamma_fpr
            .iter_mut()
            .zip(u_current.iter())
            .zip(cache.u_half_step.iter())
            .for_each(|((fpr, u), uhalf)| *fpr = u - uhalf);
        // compute the norm of FPR
        cache.norm_gamma_fpr = matrix_operations::norm2(&cache.gamma_fpr);
    }

    /// Computes a gradient step; does not compute the gradient
    fn gradient_step(&mut self, u_current: &[f64]) {
        // take a gradient step:
        // gradient_step ← u_current - gamma * gradient
        let cache = &mut self.cache;
        let gamma = cache.gamma;
        cache
            .gradient_step
            .iter_mut()
            .zip(u_current.iter())
            .zip(cache.gradient_u.iter())
            .for_each(|((grad_step, u), grad)| *grad_step = *u - gamma * *grad);
    }

    /// Takes a gradient step on u_plus
    fn gradient_step_uplus(&mut self) {
        // take a gradient step:
        // gradient_step ← u_plus - gamma * gradient
        let cache = &mut self.cache;
        let gamma = cache.gamma;
        cache
            .gradient_step
            .iter_mut()
            .zip(cache.u_plus.iter())
            .zip(cache.gradient_u.iter())
            .for_each(|((grad_step, u), grad)| *grad_step = *u - gamma * *grad);
    }

    /// Computes a projection on `gradient_step`
    fn half_step(&mut self) {
        let cache = &mut self.cache;
        // u_half_step ← projection(gradient_step)
        cache.u_half_step.copy_from_slice(&cache.gradient_step);
        self.problem.constraints.project(&mut cache.u_half_step);
    }

    /// Computes an LBFGS direction; updates `cache.direction_lbfgs`
    fn lbfgs_direction(&mut self, u_current: &[f64]) {
        let cache = &mut self.cache;
        // update the LBFGS buffer
        cache.lbfgs.update_hessian(&cache.gamma_fpr, u_current);

        // direction ← fpr
        if cache.iteration > 0 {
            cache.direction_lbfgs.copy_from_slice(&cache.gamma_fpr);
            // compute an LBFGS direction, that is direction ← H(fpr)
            cache.lbfgs.apply_hessian(&mut cache.direction_lbfgs);
        }
    }

    /// Returns the RHS of the Lipschitz update
    /// Computes rhs = cost + LIP_EPS * |f| - gamma * <gradfx, fpr> + (L/2/gamma) ||gamma * fpr||^2
    fn lipschitz_check_rhs(&mut self) -> f64 {
        let cache = &mut self.cache;
        let gamma = cache.gamma;
        let cost_value = cache.cost_value;
        // inner_prod_grad_fpr ← <gradfx, gamma_fpr>
        let inner_prod_grad_fpr =
            matrix_operations::inner_product(&cache.gradient_u, &cache.gamma_fpr);

        // rhs ← cost + LIP_EPS * |f| - <gradfx, gamma_fpr> + (L/2/gamma) ||gamma_fpr||^2
        cost_value + LIPSCHITZ_UPDATE_EPSILON * cost_value.abs() - inner_prod_grad_fpr
            + (GAMMA_L_COEFF / (2.0 * gamma)) * (cache.norm_gamma_fpr.powi(2))
    }

    /// Updates the estimate of the Lipscthiz constant
    fn update_lipschitz_constant(&mut self, u_current: &[f64]) -> FunctionCallResult {
        let mut cost_u_half_step = 0.0;

        // Compute the cost at the half step
        (self.problem.cost)(&self.cache.u_half_step, &mut cost_u_half_step)?;

        // Compute the cost at u_current (save it in `cache.cost_value`)
        (self.problem.cost)(u_current, &mut self.cache.cost_value)?;

        let mut it_lipschitz_search = 0;

        while cost_u_half_step > self.lipschitz_check_rhs()
            && it_lipschitz_search < MAX_LIPSCHITZ_UPDATE_ITERATIONS
            && self.cache.lipschitz_constant < MAX_LIPSCHITZ_CONSTANT
        {
            self.cache.lbfgs.reset(); // invalidate the L-BFGS buffer

            // update L, sigma and gamma...
            self.cache.lipschitz_constant *= 2.;
            self.cache.gamma /= 2.;

            // recompute the half step...
            self.gradient_step(u_current); // updates self.cache.gradient_step
            self.half_step(); // updates self.cache.u_half_step

            // recompute the cost at the half step
            // update `cost_u_half_step`
            (self.problem.cost)(&self.cache.u_half_step, &mut cost_u_half_step)?;

            // recompute the FPR and the square of its norm
            self.compute_fpr(u_current);
            it_lipschitz_search += 1;
        }
        self.cache.sigma = (1.0 - GAMMA_L_COEFF) / (4.0 * self.cache.gamma);

        Ok(())
    }

    /// Computes u_plus ← u - gamma * (1-tau) * fpr - tau * dir,
    fn compute_u_plus(&mut self, u: &[f64]) {
        let cache = &mut self.cache;
        let _gamma = cache.gamma;
        let tau = cache.tau;
        let temp_ = 1.0 - tau;
        cache
            .u_plus
            .iter_mut()
            .zip(u.iter())
            .zip(cache.gamma_fpr.iter())
            .zip(cache.direction_lbfgs.iter())
            .for_each(|(((u_plus_i, &u_i), &fpr_i), &dir_i)| {
                *u_plus_i = u_i - temp_ * fpr_i - tau * dir_i;
            });
    }

    /// Computes the RHS of the linesearch condition
    fn compute_rhs_ls(&mut self) {
        let cache = &mut self.cache;

        // dist squared ← norm(gradient step - u half step)^2
        let dist_squared =
            matrix_operations::norm2_squared_diff(&cache.gradient_step, &cache.u_half_step);

        // rhs_ls ← f - (gamma/2) * norm(gradf)^2
        //            + 0.5 * dist squared / gamma
        //            - sigma * norm_gamma_fpr^2
        let fbe = cache.cost_value
            - 0.5 * cache.gamma * matrix_operations::norm2_squared(&cache.gradient_u)
            + 0.5 * dist_squared / cache.gamma;
        let sigma_fpr_sq = cache.sigma * cache.norm_gamma_fpr.powi(2);
        cache.rhs_ls = fbe - sigma_fpr_sq;
    }

    /// Computes the left hand side of the line search condition and compares it with the RHS;
    /// returns `true` if and only if lhs > rhs (when the line search should continue)
    fn line_search_condition(&mut self, u: &[f64]) -> Result<bool, SolverError> {
        let gamma = self.cache.gamma;

        // u_plus ← u - (1-tau)*gamma_fpr + tau*direction
        self.compute_u_plus(&u);

        // Note: Here `cache.cost_value` and `cache.gradient_u` are overwritten
        // with the values of the cost and its gradient at the next (candidate)
        // point `u_plus`
        (self.problem.cost)(&self.cache.u_plus, &mut self.cache.cost_value)?;
        (self.problem.gradf)(&self.cache.u_plus, &mut self.cache.gradient_u)?;

        self.gradient_step_uplus(); // gradient_step ← u_plus - gamma * gradient_u
        self.half_step(); // u_half_step ← project(gradient_step)

        // Compute: dist_squared ← norm(gradient_step - u_half_step)^2
        let dist_squared = matrix_operations::norm2_squared_diff(
            &self.cache.gradient_step,
            &self.cache.u_half_step,
        );

        // Update the LHS of the line search condition
        self.cache.lhs_ls = self.cache.cost_value
            - 0.5 * gamma * matrix_operations::norm2_squared(&self.cache.gradient_u)
            + 0.5 * dist_squared / self.cache.gamma;

        Ok(self.cache.lhs_ls > self.cache.rhs_ls)
    }

    /// Update without performing a line search; this is executed at the first iteration
    fn update_no_linesearch(&mut self, u_current: &mut [f64]) -> FunctionCallResult {
        u_current.copy_from_slice(&self.cache.u_half_step); // set u_current ← u_half_step
        (self.problem.cost)(u_current, &mut self.cache.cost_value)?; // cost value
        (self.problem.gradf)(u_current, &mut self.cache.gradient_u)?; // compute gradient
        self.gradient_step(u_current); // updated self.cache.gradient_step
        self.half_step(); // updates self.cache.u_half_step

        Ok(())
    }

    /// Performs a line search to select tau
    fn linesearch(&mut self, u_current: &mut [f64]) -> FunctionCallResult {
        // perform line search
        self.compute_rhs_ls(); // compute the right hand side of the line search
        self.cache.tau = 1.0; // initialise tau ← 1.0
        let mut num_ls_iters = 0;
        while self.line_search_condition(u_current)? && num_ls_iters < MAX_LINESEARCH_ITERATIONS {
            self.cache.tau /= 2.0;
            num_ls_iters += 1;
        }
        if num_ls_iters == MAX_LINESEARCH_ITERATIONS {
            self.cache.tau = 0.;
            u_current.copy_from_slice(&self.cache.u_half_step);
        }
        // Sets `u_current` to `u_plus` (u_current ← u_plus)
        u_current.copy_from_slice(&self.cache.u_plus);

        Ok(())
    }
}

/// Implementation of the `step` and `init` methods of [trait.AlgorithmEngine.html]
impl<'a, GradientType, ConstraintType, CostType> AlgorithmEngine
    for PANOCEngine<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> FunctionCallResult,
    CostType: Fn(&[f64], &mut f64) -> FunctionCallResult,
    ConstraintType: constraints::Constraint,
{
    /// PANOC step
    ///
    /// Performs a step of PANOC, including the line search
    ///
    /// ## Arguments
    ///
    /// - `u_current` on entry is the current iterate; on exit, it is updated with the next
    ///   iterate of PANOC
    ///
    ///
    fn step(&mut self, u_current: &mut [f64]) -> Result<bool, SolverError> {
        // caches the previous gradient vector (copies df to df_previous)
        self.cache.cache_previous_gradient();

        // compute the fixed point residual
        self.compute_fpr(u_current);

        // exit if the exit conditions are satisfied (||gamma*fpr|| < eps and,
        // if activated, ||gamma*r + df - df_prev|| < eps_akkt)
        if self.cache.exit_condition() {
            return Ok(false);
        }
        self.update_lipschitz_constant(u_current)?; // update lipschitz constant
        self.lbfgs_direction(u_current); // compute LBFGS direction (update LBFGS buffer)
        if self.cache.iteration == 0 {
            // first iteration, no line search is performed
            self.update_no_linesearch(u_current)?;
        } else {
            self.linesearch(u_current)?;
        }

        self.cache.iteration += 1;
        Ok(true)
    }

    /// Initialization of PANOC
    ///
    /// Computes a number of essential quantities before the start of PANOC iterations
    ///
    /// There include the computation of the cost and gradient of the cost at the initial
    /// point, the computation of an initial estimation of the Lipschitz constant of the
    /// gradient of the cost at the initial point, initial estimates for `gamma` and `sigma`,
    /// a gradient step and a half step (projected gradient step)
    ///
    fn init(&mut self, u_current: &mut [f64]) -> FunctionCallResult {
        self.cache.reset();
        (self.problem.cost)(u_current, &mut self.cache.cost_value)?; // cost value
        self.estimate_loc_lip(u_current)?; // computes the gradient as well! (self.cache.gradient_u)
        self.cache.gamma = GAMMA_L_COEFF / self.cache.lipschitz_constant;
        self.cache.sigma = (1.0 - GAMMA_L_COEFF) / (4.0 * self.cache.gamma);
        self.gradient_step(u_current); // updated self.cache.gradient_step
        self.half_step(); // updates self.cache.u_half_step

        Ok(())
    }
}

/* --------------------------------------------------------------------------------------------- */
/*       TESTS                                                                                   */
/* --------------------------------------------------------------------------------------------- */
#[cfg(test)]
mod tests {

    use crate::constraints;
    use crate::core::panoc::panoc_engine::PANOCEngine;
    use crate::core::panoc::*;
    use crate::core::Problem;
    use crate::mocks;

    #[test]
    fn t_compute_fpr() {
        let n = 2;
        let mem = 5;
        let box_constraints = constraints::NoConstraints::new();
        let problem = Problem::new(&box_constraints, mocks::my_gradient, mocks::my_cost);
        let mut panoc_cache = PANOCCache::new(n, 1e-6, mem);
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);

        let u = [0.75, -1.4];
        panoc_engine.cache.gamma = 0.0234;
        panoc_engine.cache.u_half_step.copy_from_slice(&[0.5, 2.9]);
        panoc_engine.compute_fpr(&u); // gamma_fpr ← u - u_half_step, norm_fpr ← norm(gamma_fpr)
        unit_test_utils::assert_nearly_equal_array(
            &[0.25, -4.3],
            &panoc_engine.cache.gamma_fpr,
            1e-8,
            1e-10,
            "fpr",
        );
        unit_test_utils::assert_nearly_equal(
            4.307_261_310_856_354,
            panoc_engine.cache.norm_gamma_fpr,
            1e-8,
            1e-10,
            "norm_fpr",
        );
    }

    #[test]
    fn t_gradient_step() {
        let n = 2;
        let mem = 5;
        let bounds = constraints::NoConstraints::new();
        let problem = Problem::new(&bounds, mocks::void_gradient, mocks::void_cost);
        let mut panoc_cache = PANOCCache::new(n, 1e-6, mem);
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);

        let u = [-0.11, -1.35];
        panoc_engine.cache.gamma = 0.545;
        panoc_engine.cache.gradient_u.copy_from_slice(&[1.94, 2.6]);
        panoc_engine.gradient_step(&u); // gradient_step ← u - gamma * gradient_u

        unit_test_utils::assert_nearly_equal_array(
            &[-1.1673, -2.767],
            &panoc_engine.cache.gradient_step,
            1e-8,
            1e-10,
            "panoc_engine.cache.gradient_step",
        );
    }

    #[test]
    fn t_gradient_step_uplus() {
        let n = 2;
        let mem = 5;
        let bounds = constraints::NoConstraints::new();
        let problem = Problem::new(&bounds, mocks::void_gradient, mocks::void_cost);
        let mut panoc_cache = PANOCCache::new(n, 1e-6, mem);
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);

        panoc_engine.cache.gamma = 0.789;
        panoc_engine
            .cache
            .gradient_u
            .copy_from_slice(&[45.0, -22.40]);
        panoc_engine.cache.u_plus.copy_from_slice(&[-4.90, 10.89]);
        panoc_engine.gradient_step_uplus(); // gradient_step ← u_plus - gamma * gradient

        unit_test_utils::assert_nearly_equal_array(
            &[-40.405, 28.5636],
            &panoc_engine.cache.gradient_step,
            1e-8,
            1e-10,
            "panoc_engine.cache.gradient_step",
        );
    }

    #[test]
    fn t_half_step() {
        let n = 2;
        let mem = 5;
        let bounds = constraints::Ball2::new(None, 0.5);
        let problem = Problem::new(&bounds, mocks::void_gradient, mocks::void_cost);
        let mut panoc_cache = PANOCCache::new(n, 1e-6, mem);
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);

        panoc_engine
            .cache
            .gradient_step
            .copy_from_slice(&[40., 50.]);

        panoc_engine.half_step(); // u_half_step ← projection(gradient_step)

        unit_test_utils::assert_nearly_equal_array(
            &[0.312_347_523_777_212, 0.390_434_404_721_515],
            &panoc_engine.cache.u_half_step,
            1e-8,
            1e-10,
            "panoc_engine.cache.u_half_step",
        );
    }

    #[test]
    fn t_lipschitz_update_rhs() {
        let n = 3;
        let mem = 5;
        let bounds = constraints::NoConstraints::new();
        let problem = Problem::new(&bounds, mocks::void_gradient, mocks::void_cost);
        let mut panoc_cache = PANOCCache::new(n, 1e-6, mem);
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);

        // u = [-0.13, -0.12, -0.10];
        // gradient -- correspond to `hard_quadratic_gradient`
        panoc_engine
            .cache
            .gradient_u
            .copy_from_slice(&[-3.49, -2.35, -103.85]);

        // the following Lipschitz constant is valid for `hard_quadratic_gradient`
        panoc_engine.cache.lipschitz_constant = 2_001.305_974_987_387;

        // gamma = 0.95/L
        panoc_engine.cache.gamma = 4.746_900_333_448_449e-4;

        // fpr = (u - u_half_step)/gamma
        panoc_engine.cache.gamma_fpr.copy_from_slice(&[
            -0.001_656_668_216_374,
            -0.001_115_521_578_360,
            -0.049_296_559_962_862,
        ]);
        panoc_engine.cache.norm_gamma_fpr = 0.049_337_001_957_385;

        // cost at `u`
        panoc_engine.cache.cost_value = 5.21035;

        let rhs = panoc_engine.lipschitz_check_rhs();

        println!("rhs = {}", rhs);
        println!("cache = {:#?}", panoc_engine.cache);
        unit_test_utils::assert_nearly_equal(2.518_233_435_388_051, rhs, 1e-8, 1e-10, "lip rhs");
    }

    #[test]
    fn t_compute_rhs_ls() {
        let n = 2;
        let mem = 5;
        let bounds = constraints::Ball2::new(None, 0.5);
        let problem = Problem::new(&bounds, mocks::void_gradient, mocks::void_cost);
        let mut panoc_cache = PANOCCache::new(n, 1e-6, mem);
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);

        panoc_engine
            .cache
            .gradient_step
            .copy_from_slice(&[-0.5, -0.4]);
        panoc_engine
            .cache
            .u_half_step
            .copy_from_slice(&[15.0, -14.8]);
        panoc_engine.cache.cost_value = 24.0;
        panoc_engine.cache.gamma = 2.34;
        panoc_engine.cache.gradient_u.copy_from_slice(&[2.4, -9.7]);
        panoc_engine.cache.sigma = 0.066;
        panoc_engine.cache.norm_gamma_fpr = 2.5974;

        panoc_engine.compute_rhs_ls();

        println!("rhs = {}", panoc_engine.cache.rhs_ls);

        unit_test_utils::assert_nearly_equal(
            2.373_394_267_002_398,
            panoc_engine.cache.rhs_ls,
            1e-10,
            1e-8,
            "rhs_ls is wrong",
        );
    }
}
