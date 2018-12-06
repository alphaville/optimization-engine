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

impl<'a, GradientType, ConstraintType, CostType>
    PANOCEngine<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
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
        );
        lipest
            .with_delta(DELTA_LIPSCHITZ)
            .with_epsilon(EPSILON_LIPSCHITZ);
        self.cache.lipschitz_constant = lipest.estimate_local_lipschitz();
    }

    fn compute_fpr(&mut self, u_current: &mut [f64]) {
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

    fn half_step(&mut self) {
        // u_half_step ← projection(gradient_step)
        self.cache
            .u_half_step
            .copy_from_slice(&self.cache.gradient_step);
        self.problem
            .constraints
            .project(&mut self.cache.u_half_step);
    }

    fn lbfgs_direction(&mut self, u_current: &mut [f64]) {
        // update the LBFGS buffer
        self.cache
            .lbfgs
            .update_hessian(&self.cache.gradient_u, u_current, 1.0, 1e-10);
        // update LBFGS buffer
        self.cache
            .direction_lbfgs
            .copy_from_slice(&self.cache.fixed_point_residual);
        // compute an LBFGS direction
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

    fn update_lipschitz_constant(&mut self) {}

    fn line_search_condition(&mut self) -> bool {
        //let gamma = self.cache.gamma;
        //let tau = self.cache.tau;
        false
    }
}

/// Implementation of the `step` and `init` methods of [trait.AlgorithmEngine.html]
impl<'a, GradientType, ConstraintType, CostType> AlgorithmEngine
    for PANOCEngine<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
{
    /// PANOC step
    ///
    /// Performs a step of PANOC, including the line search
    fn step(&mut self, u_current: &mut [f64]) -> bool {
        // compute the fixed point residual
        self.compute_fpr(u_current);
        // compute the norm of FPR
        self.cache.norm_fpr = matrix_operations::norm2(&self.cache.fixed_point_residual);
        // exit if the norm of the fpr is adequetely small
        if self.cache.norm_fpr < self.cache.tolerance {
            return false;
        }
        // compute LBFGS direction (update LBFGS buffer)
        self.lbfgs_direction(u_current);

        // compute the right hand side of the line search
        self.compute_rhs_ls();

        // perform line search
        self.cache.tau = 1.0;
        while self.line_search_condition() {}
        false
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

#[cfg(test)]
mod tests {

    use super::super::*;
    use super::*;

    const N_DIM: usize = 2;

    fn assert_ae(x: f64, y: f64, tol: f64) {
        assert!((x - y).abs() < tol);
    }

    fn assert_array_ae(x: &[f64], y: &[f64], tol: f64) {
        x.iter()
            .zip(y.iter())
            .for_each(|(xi, yi)| assert!((*xi - *yi).abs() < tol));
    }

    fn my_cost(u: &[f64], cost: &mut f64) -> i32 {
        *cost = 0.5 * (u[0].powi(2) + 2. * u[1].powi(2) + 2.0 * u[0] * u[1]) + u[0] - u[1] + 3.0;
        0
    }

    fn my_gradient(u: &[f64], grad: &mut [f64]) -> i32 {
        grad[0] = u[0] + u[1] + 1.0;
        grad[1] = u[0] + 2. * u[1] - 1.0;
        0
    }

    #[test]
    fn panoc_init() {
        let radius = 0.2;
        let box_constraints = constraints::Ball2::new_at_origin_with_radius(radius);
        let problem = Problem::new(box_constraints, my_gradient, my_cost);
        let mut panoc_cache = PANOCCache::new(N_DIM, 1e-6, 5);
        {
            let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);
            let mut u = [0.75, -1.4];
            panoc_engine.init(&mut u);
            assert_ae(
                2.549509967743775,
                panoc_engine.cache.lipschitz_constant,
                1e-10,
            );
            assert_ae(0.372620625931781, panoc_engine.cache.gamma, 1e-10);
            assert_ae(0.009129205335329, panoc_engine.cache.sigma, 1e-10);
            assert_ae(6.34125, panoc_engine.cache.cost_value, 1e-10);
            assert_array_ae(&[0.35, -3.05], &panoc_engine.cache.gradient_u, 1e-10);
            assert_array_ae(
                &[0.619582780923877, -0.263507090908068],
                &panoc_engine.cache.gradient_step,
                1e-10,
            );

            assert_array_ae(
                &[0.184046458737518, -0.078274523481010],
                &panoc_engine.cache.u_half_step,
                1e-8,
            );

            assert_array_ae(&[0.75, -1.4], &u, 1e-9);
        }
        println!("cache = {:#?}", &panoc_cache);
    }

    #[test]
    fn compute_fpr() {
        let radius = 0.2;
        let box_constraints = constraints::Ball2::new_at_origin_with_radius(radius);
        let problem = Problem::new(box_constraints, my_gradient, my_cost);
        let mut panoc_cache = PANOCCache::new(N_DIM, 1e-6, 5);
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);
        let mut u = [0.75, -1.4];
        panoc_engine.init(&mut u);
        panoc_engine.compute_fpr(&mut u);
        assert_array_ae(
            &[1.518846520766933, -3.547107660006376],
            &panoc_engine.cache.fixed_point_residual,
            1e-9,
        );
    }
}
