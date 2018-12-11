use super::super::AlgorithmEngine;
use super::super::Problem;
use super::PANOCCache;
use super::PANOCEngine;
use crate::constraints;
use crate::matrix_operations;

const GAMMA_L_COEFF: f64 = 0.95;
const SIGMA_COEFF: f64 = 0.49;
const DELTA_LIPSCHITZ: f64 = 1e-10;
const EPSILON_LIPSCHITZ: f64 = 1e-10;
const LIPSCHITZ_UPDATE_EPSILON: f64 = 1e-6;

impl<'a, GradientType, ConstraintType, CostType>
    PANOCEngine<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    CostType: Fn(&[f64], &mut f64) -> i32,
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
        problem: Problem<GradientType, ConstraintType, CostType>,
        cache: &'a mut PANOCCache,
    ) -> PANOCEngine<'a, GradientType, ConstraintType, CostType> {
        PANOCEngine {
            problem: problem,
            cache: cache,
        }
    }

    fn estimate_loc_lip(&mut self, u: &mut [f64]) {
        let mut lipest = crate::lipschitz_estimator::LipschitzEstimator::new(
            u,
            &self.problem.gradf,
            &mut self.cache.gradient_u,
        )
        .with_delta(DELTA_LIPSCHITZ)
        .with_epsilon(EPSILON_LIPSCHITZ);
        self.cache.lipschitz_constant = lipest.estimate_local_lipschitz();
    }

    /// Computes the FPR and its norm
    fn compute_fpr(&mut self, u_current: &[f64]) {
        // compute the FPR:
        // fpr ← (u - u_half_step) / gamma
        let cache = &mut self.cache;
        let gamma = cache.gamma;
        cache
            .fixed_point_residual
            .iter_mut()
            .zip(u_current.iter())
            .zip(cache.u_half_step.iter())
            .for_each(|((fpr, u), uhalf)| *fpr = (u - uhalf) / gamma);
        // compute the norm of FPR
        cache.norm_fpr = matrix_operations::norm2(&cache.fixed_point_residual);
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

    /// Computes an LBFGS direction
    fn lbfgs_direction(&mut self, u_current: &[f64]) {
        let cache = &mut self.cache;
        // update the LBFGS buffer
        cache
            .lbfgs
            .update_hessian(&cache.fixed_point_residual, u_current);

        // direction ← fpr
        cache
            .direction_lbfgs
            .copy_from_slice(&cache.fixed_point_residual);
        // compute an LBFGS direction, that is direction ← H(fpr)
        cache.lbfgs.apply_hessian(&mut cache.direction_lbfgs);
    }

    /// Computes the RHS of the linesearch condition
    fn compute_rhs_ls(&mut self) {
        let cache = &mut self.cache;

        // dist squared ← norm(gradient step - u half step)^2
        let dist_squared =
            matrix_operations::norm2_squared_diff(&cache.gradient_step, &cache.u_half_step);
        // rhs_ls ← f - (gamma/2) * norm(gradf)^2 + dist squared - sigma * norm_fpr_squared
        cache.rhs_ls = cache.cost_value
            - 0.5 * cache.gamma * matrix_operations::norm2_squared(&cache.gradient_u)
            + dist_squared
            - cache.sigma * cache.norm_fpr.powi(2);
    }

    fn lipschitz_check_rhs(&mut self) -> f64 {
        let cache = &mut self.cache;
        let gamma = cache.gamma;
        let inner_prod_grad_fpr =
            matrix_operations::inner_product(&cache.gradient_u, &cache.fixed_point_residual);
        let rhs = (1.0 + LIPSCHITZ_UPDATE_EPSILON) * (cache.cost_value)
            - (gamma * inner_prod_grad_fpr)
            + (GAMMA_L_COEFF / (2.0 * gamma)) * (gamma.powi(2) * cache.norm_fpr.powi(2));
        rhs
    }

    fn update_lipschitz_constant(&mut self, u_current: &[f64]) {
        let mut cost_u_half_step = 0.0;

        // Compute the cost at the half step
        (self.problem.cost)(&self.cache.u_half_step, &mut cost_u_half_step);
        (self.problem.cost)(u_current, &mut self.cache.cost_value);

        let mut it = 0;

        while cost_u_half_step > self.lipschitz_check_rhs()
            && it < 20
            && self.cache.lipschitz_constant < 1e4
        {
            self.cache.lbfgs.reset(); // invalidate the L-BFGS buffer

            // update L, sigma and gamma...
            self.cache.lipschitz_constant *= 2.;
            self.cache.sigma /= 2.;
            self.cache.gamma /= 2.;

            // recompute the half step...
            self.gradient_step(u_current); // updates self.cache.gradient_step
            self.half_step(); // updates self.cache.u_half_step

            // recompute the cost at the half step
            // update `cost_u_half_step`
            (self.problem.cost)(&self.cache.u_half_step, &mut cost_u_half_step);

            // recompute the FPR and the square of its norm
            self.compute_fpr(u_current);
            it = it + 1;
        }
    }

    /// Computes u_plus = u - gamma * (1-tau) * fpr - tau * dir,
    fn compute_u_plus(&mut self, u: &[f64]) {
        let cache = &mut self.cache;
        let gamma = cache.gamma;
        let tau = cache.tau;
        let temp_ = (1.0 - tau) * gamma;
        cache
            .u_plus
            .iter_mut()
            .zip(u.iter())
            .zip(cache.fixed_point_residual.iter())
            .zip(cache.direction_lbfgs.iter())
            .for_each(|(((u_plus_i, &u_i), &fpr_i), &dir_i)| {
                *u_plus_i = u_i - temp_ * fpr_i - tau * gamma * dir_i;
            });
    }

    /// Computes the left hand side of the line search condition and compares it with the RHS;
    /// returns `true` if and only if lhs > rhs (when the line search should continue)
    fn line_search_condition(&mut self, u: &[f64]) -> bool {
        let gamma = self.cache.gamma;

        // u_plus ← u - (1-tau)*gamma*fpr + tau*direction
        self.compute_u_plus(&u);

        // Note: Here `cache.cost_value` and `cache.gradient_u` are overwritten
        // with the values of the cost and its gradient at the next (candidate)
        // point `u_plus`
        (self.problem.cost)(&self.cache.u_plus, &mut self.cache.cost_value);
        (self.problem.gradf)(&self.cache.u_plus, &mut self.cache.gradient_u);

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
            + dist_squared;

        self.cache.lhs_ls > self.cache.rhs_ls
    }

    fn swap_u_plus(&mut self, u_current: &mut [f64]) {
        u_current.copy_from_slice(&self.cache.u_plus);
    }
}

/// Implementation of the `step` and `init` methods of [trait.AlgorithmEngine.html]
impl<'a, GradientType, ConstraintType, CostType> AlgorithmEngine
    for PANOCEngine<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    CostType: Fn(&[f64], &mut f64) -> i32,
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
    fn step(&mut self, u_current: &mut [f64]) -> bool {
        // compute the fixed point residual
        self.compute_fpr(u_current);

        // exit if the norm of the fpr is adequetely small
        if self.cache.norm_fpr < self.cache.tolerance {
            //TODO: u <- self.cache.u_half_step
            return false;
        }

        self.update_lipschitz_constant(u_current); // update lipschitz constant

        self.lbfgs_direction(u_current); // compute LBFGS direction (update LBFGS buffer)

        if self.cache.iteration == 0 {
            // first iteration, no line search is performed
            u_current.copy_from_slice(&self.cache.u_half_step); // set u_current = u_half_step
            (self.problem.cost)(u_current, &mut self.cache.cost_value); // cost value
            (self.problem.gradf)(u_current, &mut self.cache.gradient_u); // compute gradient
            self.gradient_step(u_current); // updated self.cache.gradient_step
            self.half_step(); // updates self.cache.u_half_step
            self.cache.iteration += 1;
            return true; // continue iterating
        } else {
            // perform line search
            self.compute_rhs_ls(); // compute the right hand side of the line search
            self.cache.tau = 1.0; // initialise tau
            while self.line_search_condition(u_current) && self.cache.tau > 1e-3 {
                self.cache.tau /= 2.0;
            }
        }

        self.swap_u_plus(u_current);
        self.cache.iteration += 1;
        true
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
    fn init(&mut self, u_current: &mut [f64]) {
        (self.problem.cost)(u_current, &mut self.cache.cost_value); // cost value
        self.estimate_loc_lip(u_current); // computes the gradient as well! (self.cache.gradient_u)
        self.cache.gamma = GAMMA_L_COEFF / self.cache.lipschitz_constant;
        self.cache.sigma = (1.0 - GAMMA_L_COEFF) * SIGMA_COEFF * self.cache.gamma;
        self.gradient_step(u_current); // updated self.cache.gradient_step
        self.half_step(); // updates self.cache.u_half_step
    }
}

/* --------------------------------------------------------------------------------------------- */
/*       TESTS                                                                                   */
/* --------------------------------------------------------------------------------------------- */
#[cfg(test)]
mod tests {

    use crate::core::panoc::*;
    use crate::mocks;
    use std::num::NonZeroUsize;

    #[test]
    fn compute_fpr() {
        let n = NonZeroUsize::new(2).unwrap();
        let mem = NonZeroUsize::new(5).unwrap();
        let box_constraints = constraints::NoConstraints::new();
        let problem = Problem::new(box_constraints, mocks::my_gradient, mocks::my_cost);
        let mut panoc_cache = PANOCCache::new(n, 1e-6, mem);
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);

        let mut u = [0.75, -1.4];
        panoc_engine.cache.gamma = 0.0234;
        panoc_engine.cache.u_half_step.copy_from_slice(&[0.5, 2.9]);
        panoc_engine.compute_fpr(&mut u); // fpr ← (u - u_half_step) / gamma, norm_fpr ← norm(fpr)
        unit_test_utils::assert_nearly_equal_array(
            &[10.683760683760683, -183.7606837606838],
            &panoc_engine.cache.fixed_point_residual,
            1e-8,
            1e-10,
            "fpr",
        );
        unit_test_utils::assert_nearly_equal(
            184.0709961904425,
            panoc_engine.cache.norm_fpr,
            1e-8,
            1e-10,
            "norm_fpr",
        );
    }

    #[test]
    fn gradient_step() {
        let n = NonZeroUsize::new(2).unwrap();
        let mem = NonZeroUsize::new(5).unwrap();
        let bounds = constraints::NoConstraints::new();
        let problem = Problem::new(bounds, mocks::void_gradient, mocks::void_cost);
        let mut panoc_cache = PANOCCache::new(n, 1e-6, mem);
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);

        let mut u = [-0.11, -1.35];
        panoc_engine.cache.gamma = 0.545;
        panoc_engine.cache.gradient_u.copy_from_slice(&[1.94, 2.6]);
        panoc_engine.gradient_step(&mut u); // gradient_step ← u - gamma * gradient_u

        unit_test_utils::assert_nearly_equal_array(
            &[-1.1673, -2.767],
            &panoc_engine.cache.gradient_step,
            1e-8,
            1e-10,
            "panoc_engine.cache.gradient_step",
        );
    }

    #[test]
    fn gradient_step_uplus() {
        let n = NonZeroUsize::new(2).unwrap();
        let mem = NonZeroUsize::new(5).unwrap();
        let bounds = constraints::NoConstraints::new();
        let problem = Problem::new(bounds, mocks::void_gradient, mocks::void_cost);
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
    fn half_step() {
        let n = NonZeroUsize::new(2).unwrap();
        let mem = NonZeroUsize::new(5).unwrap();
        let bounds = constraints::Ball2::new_at_origin_with_radius(0.5);
        let problem = Problem::new(bounds, mocks::void_gradient, mocks::void_cost);
        let mut panoc_cache = PANOCCache::new(n, 1e-6, mem);
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);

        panoc_engine
            .cache
            .gradient_step
            .copy_from_slice(&[40., 50.]);

        panoc_engine.half_step(); // u_half_step ← projection(gradient_step)

        unit_test_utils::assert_nearly_equal_array(
            &[0.312347523777212, 0.390434404721515],
            &panoc_engine.cache.u_half_step,
            1e-8,
            1e-10,
            "panoc_engine.cache.u_half_step",
        );
    }

    #[test]
    fn lipschitz_update_rhs() {
        let n = NonZeroUsize::new(3).unwrap();
        let mem = NonZeroUsize::new(5).unwrap();
        let bounds = constraints::NoConstraints::new();
        let problem = Problem::new(bounds, mocks::void_gradient, mocks::void_cost);
        let mut panoc_cache = PANOCCache::new(n, 1e-6, mem);
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);

        // u = [-0.13, -0.12, -0.10];
        // gradient -- correspond to `hard_quadratic_gradient`
        panoc_engine
            .cache
            .gradient_u
            .copy_from_slice(&[-3.49, -2.35, -103.85]);

        // the following Lipschitz constant is valid for `hard_quadratic_gradient`
        panoc_engine.cache.lipschitz_constant = 2001.305974987387;

        // gamma = 0.95/L
        panoc_engine.cache.gamma = 4.746900333448449e-4;

        // fpr = (u - u_half_step)/gamma
        panoc_engine
            .cache
            .fixed_point_residual
            .copy_from_slice(&[-3.49, -2.35, -103.85]);
        panoc_engine.cache.norm_fpr = 103.9351966371354;

        // cost at `u`
        panoc_engine.cache.cost_value = 5.21035;

        let rhs = panoc_engine.lipschitz_check_rhs();

        println!("rhs = {}", rhs);
        println!("cache = {:#?}", panoc_engine.cache);
        unit_test_utils::assert_nearly_equal(2.518233435388051, rhs, 1e-8, 1e-10, "lip rhs");
    }

    #[test]
    fn compute_rhs_ls() {
        let n = NonZeroUsize::new(2).unwrap();
        let mem = NonZeroUsize::new(5).unwrap();
        let bounds = constraints::Ball2::new_at_origin_with_radius(0.5);
        let problem = Problem::new(bounds, mocks::void_gradient, mocks::void_cost);
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
        panoc_engine.cache.norm_fpr = 1.11;

        panoc_engine.compute_rhs_ls();

        println!("rhs = {}", panoc_engine.cache.rhs_ls);

        unit_test_utils::assert_nearly_equal(
            354.7041814000001,
            panoc_engine.cache.rhs_ls,
            1e-10,
            1e-8,
            "rhs_ls",
        );
    }
}
