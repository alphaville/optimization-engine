use crate::{
    constraints,
    core::{panoc::PANOCCache, AlgorithmEngine, Problem},
    matrix_operations,
    numeric::cast,
    FunctionCallResult, SolverError,
};
use lbfgs::LbfgsPrecision;
use num::Float;
use std::iter::Sum;

fn min_l_estimate<T: Float>() -> T {
    cast::<T>(1e-10)
}

fn gamma_l_coeff<T: Float>() -> T {
    cast::<T>(0.95)
}

//const SIGMA_COEFF: f64 = 0.49;

fn delta_lipschitz<T: Float>() -> T {
    cast::<T>(1e-12)
}

fn epsilon_lipschitz<T: Float>() -> T {
    cast::<T>(1e-6)
}

fn lipschitz_update_epsilon<T: Float>() -> T {
    cast::<T>(1e-6)
}

/// Maximum iterations of updating the Lipschitz constant
const MAX_LIPSCHITZ_UPDATE_ITERATIONS: usize = 10;

fn max_lipschitz_constant<T: Float>() -> T {
    cast::<T>(1e9)
}

fn norm2_squared_diff<T: Float>(a: &[T], b: &[T]) -> T {
    assert_eq!(a.len(), b.len());
    a.iter()
        .zip(b.iter())
        .fold(T::zero(), |sum, (&x, &y)| sum + (x - y) * (x - y))
}

/// Maximum number of linesearch iterations
const MAX_LINESEARCH_ITERATIONS: u32 = 10;

/// Engine for PANOC algorithm
pub struct PANOCEngine<'a, GradientType, ConstraintType, CostType, T = f64>
where
    T: Float + LbfgsPrecision + Sum<T>,
    GradientType: Fn(&[T], &mut [T]) -> FunctionCallResult,
    CostType: Fn(&[T], &mut T) -> FunctionCallResult,
    ConstraintType: constraints::Constraint<T>,
{
    problem: Problem<'a, GradientType, ConstraintType, CostType, T>,
    pub(crate) cache: &'a mut PANOCCache<T>,
}

impl<'a, GradientType, ConstraintType, CostType, T>
    PANOCEngine<'a, GradientType, ConstraintType, CostType, T>
where
    T: Float + LbfgsPrecision + Sum<T>,
    GradientType: Fn(&[T], &mut [T]) -> FunctionCallResult,
    CostType: Fn(&[T], &mut T) -> FunctionCallResult,
    ConstraintType: constraints::Constraint<T>,
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
        problem: Problem<'a, GradientType, ConstraintType, CostType, T>,
        cache: &'a mut PANOCCache<T>,
    ) -> PANOCEngine<'a, GradientType, ConstraintType, CostType, T> {
        PANOCEngine { problem, cache }
    }

    /// Estimate the local Lipschitz constant at `u`
    fn estimate_loc_lip(&mut self, u: &mut [T]) -> FunctionCallResult {
        let mut lipest = crate::lipschitz_estimator::LipschitzEstimator::new(
            u,
            &self.problem.gradf,
            &mut self.cache.gradient_u,
        )
        .with_delta(delta_lipschitz())
        .with_epsilon(epsilon_lipschitz());
        self.cache.lipschitz_constant = lipest.estimate_local_lipschitz()?;

        Ok(())
    }

    /// Computes the FPR and its norm
    fn compute_fpr(&mut self, u_current: &[T]) {
        // compute the FPR:
        // fpr ← u - u_half_step
        let cache = &mut self.cache;
        cache
            .gamma_fpr
            .iter_mut()
            .zip(u_current.iter())
            .zip(cache.u_half_step.iter())
            .for_each(|((fpr, u), uhalf)| *fpr = *u - *uhalf);
        // compute the norm of FPR
        cache.norm_gamma_fpr = matrix_operations::norm2(&cache.gamma_fpr);
    }

    /// Score the current feasible half step and cache it if it is the best so far.
    pub(crate) fn cache_best_half_step(&mut self, u_current: &[T]) {
        self.compute_fpr(u_current);
        self.cache.cache_best_half_step();
    }

    /// Computes a gradient step; does not compute the gradient
    fn gradient_step(&mut self, u_current: &[T]) {
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

    /// Cache the squared norm of the current gradient.
    fn cache_gradient_norm(&mut self) {
        self.cache.gradient_u_norm_sq = matrix_operations::norm2_squared(&self.cache.gradient_u);
    }

    /// Computes a projection on `gradient_step`
    fn half_step(&mut self) -> FunctionCallResult {
        let cache = &mut self.cache;
        // u_half_step ← projection(gradient_step)
        cache.u_half_step.copy_from_slice(&cache.gradient_step);
        self.problem.constraints.project(&mut cache.u_half_step)?;
        cache.gradient_step_u_half_step_diff_norm_sq =
            norm2_squared_diff(&cache.gradient_step, &cache.u_half_step);
        Ok(())
    }

    /// Computes an LBFGS direction; updates `cache.direction_lbfgs`
    fn lbfgs_direction(&mut self, u_current: &[T]) {
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
    fn lipschitz_check_rhs(&mut self) -> T {
        let cache = &mut self.cache;
        let gamma = cache.gamma;
        let cost_value = cache.cost_value;
        // inner_prod_grad_fpr ← <gradfx, gamma_fpr>
        let inner_prod_grad_fpr =
            matrix_operations::inner_product(&cache.gradient_u, &cache.gamma_fpr);

        // rhs ← cost + LIP_EPS * |f| - <gradfx, gamma_fpr> + (L/2/gamma) ||gamma_fpr||^2
        cost_value + lipschitz_update_epsilon::<T>() * cost_value.abs() - inner_prod_grad_fpr
            + (gamma_l_coeff::<T>() / (cast::<T>(2.0) * gamma))
                * cache.norm_gamma_fpr
                * cache.norm_gamma_fpr
    }

    /// Updates the estimate of the Lipscthiz constant
    fn update_lipschitz_constant(&mut self, u_current: &[T]) -> FunctionCallResult {
        let mut cost_u_half_step = T::zero();

        // Compute the cost at the half step
        (self.problem.cost)(&self.cache.u_half_step, &mut cost_u_half_step)?;
        if !matrix_operations::is_finite(&[self.cache.cost_value, cost_u_half_step]) {
            return Err(SolverError::NotFiniteComputation(
                "cost evaluation returned a non-finite value during Lipschitz estimation",
            ));
        }

        let mut it_lipschitz_search = 0;

        while cost_u_half_step > self.lipschitz_check_rhs()
            && it_lipschitz_search < MAX_LIPSCHITZ_UPDATE_ITERATIONS
            && self.cache.lipschitz_constant < max_lipschitz_constant()
        {
            self.cache.lbfgs.reset(); // invalidate the L-BFGS buffer

            // update L, sigma and gamma...
            self.cache.lipschitz_constant = self.cache.lipschitz_constant * cast::<T>(2.0);
            self.cache.gamma = self.cache.gamma / cast::<T>(2.0);

            // recompute the half step...
            self.gradient_step(u_current); // updates self.cache.gradient_step
            self.half_step()?; // updates self.cache.u_half_step

            // recompute the cost at the half step
            // update `cost_u_half_step`
            (self.problem.cost)(&self.cache.u_half_step, &mut cost_u_half_step)?;
            if !cost_u_half_step.is_finite() {
                return Err(SolverError::NotFiniteComputation(
                    "half-step cost became non-finite during Lipschitz backtracking",
                ));
            }

            // recompute the FPR and the square of its norm
            self.compute_fpr(u_current);
            it_lipschitz_search += 1;
        }
        self.cache.sigma = (T::one() - gamma_l_coeff::<T>()) / (cast::<T>(4.0) * self.cache.gamma);

        Ok(())
    }

    /// Computes u_plus ← u - gamma * (1-tau) * fpr - tau * dir,
    fn compute_u_plus(&mut self, u: &[T]) {
        let cache = &mut self.cache;
        let _gamma = cache.gamma;
        let tau = cache.tau;
        let temp_ = T::one() - tau;
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
        let half = cast::<T>(0.5);

        // dist squared ← norm(gradient step - u half step)^2
        // rhs_ls ← f - (gamma/2) * norm(gradf)^2
        //            + 0.5 * dist squared / gamma
        //            - sigma * norm_gamma_fpr^2
        let fbe = cache.cost_value - half * cache.gamma * cache.gradient_u_norm_sq
            + half * cache.gradient_step_u_half_step_diff_norm_sq / cache.gamma;
        let sigma_fpr_sq = cache.sigma * cache.norm_gamma_fpr * cache.norm_gamma_fpr;
        cache.rhs_ls = fbe - sigma_fpr_sq;
    }

    /// Computes the left hand side of the line search condition and compares it with the RHS;
    /// returns `true` if and only if lhs > rhs (when the line search should continue)
    fn line_search_condition(&mut self, u: &[T]) -> Result<bool, SolverError> {
        let gamma = self.cache.gamma;
        let half = cast::<T>(0.5);

        // u_plus ← u - (1-tau)*gamma_fpr + tau*direction
        self.compute_u_plus(u);

        // Note: Here `cache.cost_value` and `cache.gradient_u` are overwritten
        // with the values of the cost and its gradient at the next (candidate)
        // point `u_plus`
        (self.problem.cost)(&self.cache.u_plus, &mut self.cache.cost_value)?;
        (self.problem.gradf)(&self.cache.u_plus, &mut self.cache.gradient_u)?;
        if !self.cache.cost_value.is_finite()
            || !matrix_operations::is_finite(&self.cache.gradient_u)
        {
            return Err(SolverError::NotFiniteComputation(
                "line-search candidate produced a non-finite cost or gradient",
            ));
        }
        self.cache_gradient_norm();

        self.gradient_step_uplus(); // gradient_step ← u_plus - gamma * gradient_u
        self.half_step()?; // u_half_step ← project(gradient_step)

        // Update the LHS of the line search condition
        self.cache.lhs_ls = self.cache.cost_value - half * gamma * self.cache.gradient_u_norm_sq
            + half * self.cache.gradient_step_u_half_step_diff_norm_sq / self.cache.gamma;

        Ok(self.cache.lhs_ls > self.cache.rhs_ls)
    }

    /// Update without performing a line search; this is executed at the first iteration
    fn update_no_linesearch(&mut self, u_current: &mut [T]) -> FunctionCallResult {
        u_current.copy_from_slice(&self.cache.u_half_step); // set u_current ← u_half_step
        (self.problem.cost)(u_current, &mut self.cache.cost_value)?; // cost value
        (self.problem.gradf)(u_current, &mut self.cache.gradient_u)?; // compute gradient
        if !self.cache.cost_value.is_finite()
            || !matrix_operations::is_finite(&self.cache.gradient_u)
        {
            return Err(SolverError::NotFiniteComputation(
                "first PANOC iterate produced a non-finite cost or gradient",
            ));
        }
        self.cache_gradient_norm();
        self.gradient_step(u_current); // updated self.cache.gradient_step
        self.half_step()?; // updates self.cache.u_half_step

        Ok(())
    }

    /// Performs a line search to select tau
    fn linesearch(&mut self, u_current: &mut [T]) -> FunctionCallResult {
        // perform line search
        self.compute_rhs_ls(); // compute the right hand side of the line search
        self.cache.tau = T::one(); // initialise tau ← 1.0
        let mut num_ls_iters = 0;
        while self.line_search_condition(u_current)? && num_ls_iters < MAX_LINESEARCH_ITERATIONS {
            self.cache.tau = self.cache.tau / cast::<T>(2.0);
            num_ls_iters += 1;
        }
        if num_ls_iters == MAX_LINESEARCH_ITERATIONS {
            self.cache.tau = T::zero();
            u_current.copy_from_slice(&self.cache.u_half_step);
        }
        // Sets `u_current` to `u_plus` (u_current ← u_plus)
        u_current.copy_from_slice(&self.cache.u_plus);

        Ok(())
    }

    /// Compute the cost value at the best cached feasible half step.
    pub(crate) fn cost_value_at_best_half_step(&mut self) -> Result<T, SolverError> {
        let mut cost = T::zero();
        (self.problem.cost)(&self.cache.best_u_half_step, &mut cost)?;
        if !cost.is_finite() {
            return Err(SolverError::NotFiniteComputation(
                "best cached half-step cost is non-finite",
            ));
        }
        Ok(cost)
    }
}

/// Implementation of the `step` and `init` methods of [trait.AlgorithmEngine.html]
impl<'a, GradientType, ConstraintType, CostType, T> AlgorithmEngine<T>
    for PANOCEngine<'a, GradientType, ConstraintType, CostType, T>
where
    T: Float + LbfgsPrecision + Sum<T>,
    GradientType: Fn(&[T], &mut [T]) -> FunctionCallResult,
    CostType: Fn(&[T], &mut T) -> FunctionCallResult,
    ConstraintType: constraints::Constraint<T>,
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
    fn step(&mut self, u_current: &mut [T]) -> Result<bool, SolverError> {
        // caches the previous gradient vector (copies df to df_previous)
        self.cache.cache_previous_gradient();

        // compute the fixed point residual
        self.cache_best_half_step(u_current);

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
    fn init(&mut self, u_current: &mut [T]) -> FunctionCallResult {
        self.cache.reset();
        (self.problem.cost)(u_current, &mut self.cache.cost_value)?; // cost value
        self.estimate_loc_lip(u_current)?; // computes the gradient as well! (self.cache.gradient_u)
        if !self.cache.cost_value.is_finite()
            || !matrix_operations::is_finite(&self.cache.gradient_u)
        {
            return Err(SolverError::NotFiniteComputation(
                "initial PANOC cost or gradient is non-finite",
            ));
        }
        self.cache_gradient_norm();
        self.cache.gamma =
            gamma_l_coeff::<T>() / self.cache.lipschitz_constant.max(min_l_estimate());
        self.cache.sigma = (T::one() - gamma_l_coeff::<T>()) / (cast::<T>(4.0) * self.cache.gamma);
        self.gradient_step(u_current); // updated self.cache.gradient_step
        self.half_step()?; // updates self.cache.u_half_step

        Ok(())
    }
}

/* --------------------------------------------------------------------------------------------- */
/*       TESTS                                                                                   */
/* --------------------------------------------------------------------------------------------- */
#[cfg(test)]
mod tests {
    use std::cell::Cell;

    use crate::constraints;
    use crate::core::panoc::panoc_engine::PANOCEngine;
    use crate::core::panoc::*;
    use crate::core::{AlgorithmEngine, Problem};
    use crate::mocks;
    use crate::FunctionCallResult;

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

        panoc_engine.half_step().unwrap(); // u_half_step ← projection(gradient_step)

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
        panoc_engine.cache.gradient_u_norm_sq =
            crate::matrix_operations::norm2_squared(&panoc_engine.cache.gradient_u);
        panoc_engine.cache.gradient_step_u_half_step_diff_norm_sq =
            crate::matrix_operations::norm2_squared_diff(
                &panoc_engine.cache.gradient_step,
                &panoc_engine.cache.u_half_step,
            );
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

    #[test]
    fn t_update_lipschitz_constant_reuses_cached_cost_at_current_iterate() {
        let n = 2;
        let mem = 5;
        let bounds = constraints::NoConstraints::new();
        let cost_calls = Cell::new(0usize);
        let cost = |u: &[f64], c: &mut f64| -> FunctionCallResult {
            cost_calls.set(cost_calls.get() + 1);
            *c = 0.5 * crate::matrix_operations::norm2_squared(u);
            Ok(())
        };
        let grad = |u: &[f64], g: &mut [f64]| -> FunctionCallResult {
            g.copy_from_slice(u);
            Ok(())
        };
        let problem = Problem::new(&bounds, grad, cost);
        let mut panoc_cache = PANOCCache::new(n, 1e-6, mem);
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);

        let u_current = [1.0, -2.0];
        panoc_engine.cache.cost_value = 2.5;
        panoc_engine.cache.u_half_step.copy_from_slice(&[0.1, -0.1]);
        panoc_engine.cache.gradient_u.copy_from_slice(&u_current);
        panoc_engine.cache.gamma = 0.5;
        panoc_engine.cache.lipschitz_constant = 1.9;
        panoc_engine.compute_fpr(&u_current);

        panoc_engine.update_lipschitz_constant(&u_current).unwrap();

        assert_eq!(
            1,
            cost_calls.get(),
            "update_lipschitz_constant should only evaluate the half-step cost"
        );
    }

    #[test]
    fn t_panoc_init_f32() {
        let bounds = constraints::NoConstraints::new();
        let problem = Problem::new(
            &bounds,
            |u: &[f32], grad: &mut [f32]| -> FunctionCallResult {
                grad.copy_from_slice(u);
                Ok(())
            },
            |u: &[f32], c: &mut f32| -> FunctionCallResult {
                *c = 0.5_f32 * (u[0] * u[0] + u[1] * u[1]);
                Ok(())
            },
        );
        let mut panoc_cache = PANOCCache::<f32>::new(2, 1e-6_f32, 3);
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);
        let mut u = [1_000.0_f32, 2_000.0_f32];

        panoc_engine.init(&mut u).unwrap();

        assert!(panoc_engine.cache.lipschitz_constant.is_finite());
        assert!(panoc_engine.cache.lipschitz_constant > 0.0_f32);
        let expected_gamma = 0.95_f32 / panoc_engine.cache.lipschitz_constant;
        assert!((panoc_engine.cache.gamma - expected_gamma).abs() < 1e-6);
        unit_test_utils::assert_nearly_equal_array(
            &[1_000.0_f32, 2_000.0_f32],
            &panoc_engine.cache.gradient_u,
            1e-5,
            1e-6,
            "gradient at u",
        );
        let expected_half_step = [
            (1.0_f32 - panoc_engine.cache.gamma) * 1_000.0_f32,
            (1.0_f32 - panoc_engine.cache.gamma) * 2_000.0_f32,
        ];
        assert!((panoc_engine.cache.u_half_step[0] - expected_half_step[0]).abs() < 5e-3);
        assert!((panoc_engine.cache.u_half_step[1] - expected_half_step[1]).abs() < 5e-3);
    }
}
