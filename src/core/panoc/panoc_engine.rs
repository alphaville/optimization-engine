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

    fn compute_fpr(&mut self, u_current: &[f64]) {
        // compute the FPR:
        // fpr ← (u - u_half_step) / gamma
        let gamma = self.cache.gamma;
        self.cache
            .fixed_point_residual
            .iter_mut()
            .zip(u_current.iter())
            .zip(self.cache.u_half_step.iter())
            .for_each(|((fpr, u), uhalf)| *fpr = (u - uhalf) / gamma);
    }

    fn gradient_step(&mut self, u_current: &[f64]) {
        // take a gradient step:
        // gradient_step ← u_current - gamma * gradient
        let gamma = self.cache.gamma;
        self.cache
            .gradient_step
            .iter_mut()
            .zip(u_current.iter())
            .zip(self.cache.gradient_u.iter())
            .for_each(|((grad_step, u), grad)| *grad_step = *u - gamma * *grad);
    }

    fn gradient_step_uplus(&mut self) {
        // take a gradient step:
        // gradient_step ← u_current - gamma * gradient
        let gamma = self.cache.gamma;
        self.cache
            .gradient_step
            .iter_mut()
            .zip(self.cache.u_plus.iter())
            .zip(self.cache.gradient_u.iter())
            .for_each(|((grad_step, u), grad)| *grad_step = *u - gamma * *grad);
    }

    fn half_step(&mut self) {
        // u_half_step ← projection(gradient_step)
        self.cache
            .u_half_step
            .copy_from_slice(&self.cache.gradient_step);
        self.problem
            .constraints
            .project(&mut self.cache.u_half_step);
    }

    fn lbfgs_direction(&mut self, u_current: &[f64]) {
        // update the LBFGS buffer
        self.cache
            .lbfgs
            .update_hessian(&self.cache.fixed_point_residual, u_current);
        // direction ← fpr
        self.cache
            .direction_lbfgs
            .copy_from_slice(&self.cache.fixed_point_residual);
        // compute an LBFGS direction, that is direction ← H(fpr)
        self.cache
            .lbfgs
            .apply_hessian(&mut self.cache.direction_lbfgs);
    }

    fn compute_rhs_ls(&mut self) {
        // dist squared ← norm(gradient step - u half step)^2
        let dist_squared = self
            .cache
            .gradient_step
            .iter()
            .zip(self.cache.u_half_step.iter())
            .map(|(g, uh)| (*g - *uh).powi(2))
            .sum::<f64>();
        // rhs_ls ← f - (gamma/2) * norm(gradf)^2 + dist squared - sigma * norm_fpr_squared
        self.cache.rhs_ls = self.cache.cost_value
            - (self.cache.gamma / 2.0) * matrix_operations::norm2(&self.cache.gradient_u)
            + dist_squared
            - self.cache.sigma
            + self.cache.norm_fpr.powi(2);
    }

    fn lipschitz_update(&mut self, cost_u_half_step: f64, norm_fpr_squared: f64) -> bool {
        cost_u_half_step
            > (1. + LIPSCHITZ_UPDATE_EPSILON) * self.cache.cost_value
                - self.cache.gamma
                    * matrix_operations::inner_product(
                        &self.cache.gradient_u,
                        &self.cache.fixed_point_residual,
                    )
                + 0.5 * self.cache.lipschitz_constant * self.cache.gamma.powi(2) * norm_fpr_squared
    }

    fn update_lipschitz_constant(&mut self, u_current: &[f64]) {
        let mut cost_u_half_step = 0.0;
        (self.problem.cost)(&self.cache.u_half_step, &mut cost_u_half_step);
        let mut norm_fpr_squared = self.cache.norm_fpr.powi(2);
        while self.lipschitz_update(cost_u_half_step, norm_fpr_squared) {
            self.cache.lipschitz_constant *= 2.0;
            self.cache.sigma /= 2.0;
            self.cache.gamma /= 2.0;
            self.gradient_step(u_current); // updated self.cache.gradient_step
            self.half_step(); // updates self.cache.u_half_step
            self.compute_fpr(u_current);
            norm_fpr_squared = matrix_operations::norm2_squared(&self.cache.fixed_point_residual);
        }
    }

    fn line_search_condition(&mut self, u: &[f64]) -> bool {
        let gamma = self.cache.gamma;
        let tau = self.cache.tau;
        // u_plus ← u - (1-tau)*gamma*fpr + tau*direction
        // Important Note: Method `lbfgs_direction` computes the direction Hk*fpr, but we
        // need Hk*(-fpr). Essentially, it computes the negative of the desired direction.
        // This is why in the code below we use `- tau * dir_i` (instead of `+ tau * dir_i`)
        let temp_ = (1.0 - tau) * gamma;
        self.cache
            .u_plus
            .iter_mut()
            .zip(u.iter())
            .zip(self.cache.fixed_point_residual.iter())
            .zip(self.cache.direction_lbfgs.iter())
            .for_each(|(((u_plus_i, &u_i), &fpr_i), &dir_i)| {
                *u_plus_i = u_i + temp_ * fpr_i - tau * dir_i;
            });
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
        u_current
            .iter_mut()
            .zip(self.cache.u_plus.iter())
            .for_each(|(ui, &uplusi)| *ui = uplusi);
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

        // compute the norm of FPR
        self.cache.norm_fpr = matrix_operations::norm2(&self.cache.fixed_point_residual);

        // exit if the norm of the fpr is adequetely small
        if self.cache.norm_fpr < self.cache.tolerance {
            return false;
        }

        // update lipschitz constant
        self.update_lipschitz_constant(u_current);

        // compute LBFGS direction (update LBFGS buffer)
        self.lbfgs_direction(u_current);

        // compute the right hand side of the line search
        self.compute_rhs_ls();

        // perform line search
        self.cache.tau = 1.0;
        while self.line_search_condition(u_current) {
            self.cache.tau /= 2.0;
        }

        self.swap_u_plus(u_current);
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

    use super::*;
    use crate::mocks;
    use std::num::NonZeroUsize;

    const N_DIM: usize = 2;

    #[test]
    fn panoc_init() {
        let radius = 0.2;
        let box_constraints = constraints::Ball2::new_at_origin_with_radius(radius);
        let problem = Problem::new(box_constraints, mocks::my_gradient, mocks::my_cost);
        let mut panoc_cache = PANOCCache::new(
            NonZeroUsize::new(N_DIM).unwrap(),
            1e-6,
            NonZeroUsize::new(5).unwrap(),
        );

        {
            let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);
            let mut u = [0.75, -1.4];
            panoc_engine.init(&mut u);
            unit_test_utils::assert_nearly_equal(
                2.549509967743775,
                panoc_engine.cache.lipschitz_constant,
                1e-4,
                1e-10,
                "lipschitz",
            );
            unit_test_utils::assert_nearly_equal(
                0.372620625931781,
                panoc_engine.cache.gamma,
                1e-4,
                1e-10,
                "gamma",
            );
            unit_test_utils::assert_nearly_equal(
                0.009129205335329,
                panoc_engine.cache.sigma,
                1e-4,
                1e-10,
                "sigma",
            );
            unit_test_utils::assert_nearly_equal(
                6.34125,
                panoc_engine.cache.cost_value,
                1e-4,
                1e-10,
                "cost value",
            );
            unit_test_utils::assert_nearly_equal_array(
                &[0.35, -3.05],
                &panoc_engine.cache.gradient_u,
                1e-4,
                1e-10,
                "gradient at u",
            );
            unit_test_utils::assert_nearly_equal_array(
                &[0.619582780923877, -0.263507090908068],
                &panoc_engine.cache.gradient_step,
                1e-4,
                1e-10,
                "gradient step",
            );

            unit_test_utils::assert_nearly_equal_array(
                &[0.184046458737518, -0.078274523481010],
                &panoc_engine.cache.u_half_step,
                1e-3,
                1e-8,
                "u_half_step",
            );

            unit_test_utils::assert_nearly_equal_array(&[0.75, -1.4], &u, 1e-4, 1e-9, "u");
        }
        println!("cache = {:#?}", &panoc_cache);
    }

    #[test]
    fn compute_fpr() {
        let radius = 0.2;
        let box_constraints = constraints::Ball2::new_at_origin_with_radius(radius);
        let problem = Problem::new(box_constraints, mocks::my_gradient, mocks::my_cost);
        let mut panoc_cache = PANOCCache::new(
            NonZeroUsize::new(N_DIM).unwrap(),
            1e-6,
            NonZeroUsize::new(5).unwrap(),
        );
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);
        let mut u = [0.75, -1.4];
        panoc_engine.init(&mut u);
        panoc_engine.compute_fpr(&mut u);
        unit_test_utils::assert_nearly_equal_array(
            &[1.518846520766933, -3.547107660006376],
            &panoc_engine.cache.fixed_point_residual,
            1e-4,
            1e-9,
            "fpr",
        );
    }

    #[test]
    fn wild_test_panoc() {
        // NOTE: this test is work in progress!!!
        let radius = 0.2;
        let box_constraints = constraints::Ball2::new_at_origin_with_radius(radius);
        let problem = Problem::new(box_constraints, mocks::my_gradient, mocks::my_cost);
        let mut panoc_cache = PANOCCache::new(
            NonZeroUsize::new(N_DIM).unwrap(),
            1e-10,
            NonZeroUsize::new(5).unwrap(),
        );
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);

        let mut u = [0.75, -1.4];
        panoc_engine.init(&mut u);
        panoc_engine.step(&mut u);
        let fpr0 = panoc_engine.cache.norm_fpr;
        println!("fpr0 = {}", fpr0);
        for _i in 1..10 {
            println!("------------------------------------------------------");
            panoc_engine.step(&mut u);
            println!("fpr = {}", panoc_engine.cache.norm_fpr);
            println!("fpr/fpr0 = {}", panoc_engine.cache.norm_fpr / fpr0);
            println!("L = {}", panoc_engine.cache.lipschitz_constant);
            println!("gamma = {}", panoc_engine.cache.gamma);
            println!("tau = {}", panoc_engine.cache.tau);
            println!("lbfgs = {:.4?}", panoc_engine.cache.direction_lbfgs);
            println!("u = {:.4?}", u);
        }
    }
}
