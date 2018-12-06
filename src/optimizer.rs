//! Optimisation algorithms
//!
//! ## Example: Forward-Backward Splitting
//!
//! ```
//! use panoc_rs::optimizer::*;
//! use panoc_rs::constraints::Ball2;
//!
//! fn my_cost(u: &[f64], cost: &mut f64) -> i32 {
//!     *cost = u[0] * u[0] + 2. * u[1] * u[1] + u[0] - u[1] + 3.0;
//!     0
//! }
//!
//! fn my_gradient(u: &[f64], grad: &mut [f64]) -> i32 {
//!     grad[0] = u[0] + u[1] + 1.0;
//!     grad[1] = u[0] + 2. * u[1] - 1.0;
//!     0
//! }
//!
//! fn main() {
//!     let radius = 0.2;
//!     let box_constraints = Ball2::new_at_origin_with_radius(radius);
//!     let problem = Problem::new(box_constraints, my_gradient, my_cost);
//!     let gamma = 0.1;
//!     let tolerance = 1e-6;
//!     let mut fbs_cache = FBSCache::new(2, gamma, tolerance);
//!     let mut fbs_step = FBSEngine::new(problem, &mut fbs_cache);
//!     let mut u = [0.0; 2];
//!     let mut optimizer = FBSOptimizer::new(&mut fbs_step);
//!     let status = optimizer.solve(&mut u);
//!     assert!(status.has_converged());
//! }
//! ```
use crate::constraints;
use crate::matrix_operations;

// default maximum number of iterations
const MAX_ITER: usize = 100_usize;
const GAMMA_L_COEFF: f64 = 0.95;
const SIGMA_COEFF: f64 = 0.49;

/* --------------------------------------------------------------------------------------------- */

/// Solver status
///
/// This structure contais information about the solver status. Instances of
/// `SolverStatus` are returned by optimizers.
///
pub struct SolverStatus {
    /// whether the algorithm has converged
    converged: bool,
    /// number of iterations for convergence
    num_iter: usize,
    /// norm of the fixed-point residual (FPR)
    fpr_norm: f64,
    /// cost value at the candidate solution
    cost_value: f64,
}

impl SolverStatus {
    /// Constructs a new instance of SolverStatus
    ///
    /// ## Arguments
    ///
    /// - `converged` whether the algorithm has converged to a solution up to
    ///   the specified tolerance
    /// - `num_iter` number of iterations
    /// - `fpr_norm` norm of the fixed-point residual; a gauge of the solution
    ///    quality
    /// - `cost_value` the value of the cost function at the solution
    ///
    pub fn new(converged: bool, num_iter: usize, fpr_norm: f64, cost_value: f64) -> SolverStatus {
        SolverStatus {
            converged: converged,
            num_iter: num_iter,
            fpr_norm: fpr_norm,
            cost_value: cost_value,
        }
    }

    /// whether the algorithm has converged
    pub fn has_converged(&self) -> bool {
        self.converged
    }

    /// number of iterations taken by the algorithm
    pub fn get_number_iterations(&self) -> usize {
        self.num_iter
    }

    /// norm of the fixed point residual
    pub fn get_norm_fpr(&self) -> f64 {
        self.fpr_norm
    }

    /// value of the cost at the solution
    pub fn get_cost_value(&self) -> f64 {
        self.cost_value
    }
}

/* --------------------------------------------------------------------------------------------- */

/// A general optimizer
pub trait Optimizer {
    /// solves a given problem and updates the initial estimate `u` with the solution
    ///
    /// Returns the solver status
    ///
    fn solve(&mut self, u: &mut [f64]) -> SolverStatus;
}

/* --------------------------------------------------------------------------------------------- */

/// Engine supporting an algorithm
///
/// An engine is responsible for the allocation of memory for an algorithm,
/// especially memory that is reuasble is multiple instances of the same
/// algorithm (as in model predictive control).
///
/// It defines what the algorithm does at every step (see `step`) and whether
/// the specified termination criterion is satisfied
///
pub trait AlgorithmEngine {
    /// Take a step of the algorithm and return `true` only if the iterations should continue
    fn step(&mut self, &mut [f64]) -> bool;

    fn init(&mut self, &mut [f64]);
}

/* --------------------------------------------------------------------------------------------- */

/// Definition of an optimisation problem
///
/// The definition of an optimisation problem involves:
/// - the gradient of the cost function
/// - the cost function
/// - the set of constraints, which is described by implementations of
///   [Constraint](../../panoc_rs/constraints/trait.Constraint.html)
pub struct Problem<GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    CostType: Fn(&[f64], &mut f64) -> i32,
    ConstraintType: constraints::Constraint,
{
    /// constraints
    constraints: ConstraintType,
    /// gradient of the cost
    gradf: GradientType,
    /// cost function
    cost: CostType,
}

impl<GradientType, ConstraintType, CostType> Problem<GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    CostType: Fn(&[f64], &mut f64) -> i32,
    ConstraintType: constraints::Constraint,
{
    /// Construct a new instance of an optimisation problem
    ///
    /// ## Arguments
    ///
    /// - `constraints` constraints
    /// - `cost_gradient` gradient of the cost function
    /// - `cost` cost function
    ///
    pub fn new(
        constraints: ConstraintType,
        cost_gradient: GradientType,
        cost: CostType,
    ) -> Problem<GradientType, ConstraintType, CostType> {
        Problem {
            constraints: constraints,
            gradf: cost_gradient,
            cost: cost,
        }
    }
}

/* --------------------------------------------------------------------------------------------- */

/// Cache for the forward-backward splitting (FBS), or projected gradient, algorithm
///
/// This struct allocates memory needed for the FBS algorithm
pub struct FBSCache {
    work_gradient_u: Vec<f64>,
    work_u_previous: Vec<f64>,
    gamma: f64,
    tolerance: f64,
    norm_fpr: f64,
}

impl FBSCache {
    /// Construct a new instance of `FBSCache`
    ///
    /// ## Arguments
    ///
    /// - `gamma` parameter gamma of the algorithm
    /// - `tolerance` tolerance used for termination
    ///
    /// ## Memory allocation
    ///
    /// This method allocates new memory (which it owns, of course). You should avoid
    /// constructing instances of `FBSCache`  in a loop or in any way more than
    /// absolutely necessary
    ///
    /// If you need to call an optimizer more than once, perhaps with different
    /// parameters, then construct an `FBSCache` only once
    ///
    pub fn new(n: usize, gamma: f64, tolerance: f64) -> FBSCache {
        FBSCache {
            work_gradient_u: vec![0.0; n],
            work_u_previous: vec![0.0; n],
            gamma: gamma,
            tolerance: tolerance,
            norm_fpr: std::f64::INFINITY,
        }
    }
}

/* --------------------------------------------------------------------------------------------- */

/// The FBE Engine defines the steps of the FBE algorithm and the termination criterion
///
pub struct FBSEngine<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
{
    problem: Problem<GradientType, ConstraintType, CostType>,
    cache: &'a mut FBSCache,
}

impl<'a, GradientType, ConstraintType, CostType>
    FBSEngine<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
{
    /// Constructor for instances of `FBSEngine`
    ///
    /// ## Arguments
    ///
    /// - `problem` problem definition (cost function, gradient of the cost, constraints)
    /// - mutable reference to a `cache` a cache (which is created once); the cache is reuseable
    ///
    /// ## Returns
    ///
    /// An new instance of `FBSEngine`
    pub fn new(
        problem: Problem<GradientType, ConstraintType, CostType>,
        cache: &'a mut FBSCache,
    ) -> FBSEngine<'a, GradientType, ConstraintType, CostType> {
        FBSEngine {
            problem: problem,
            cache: cache,
        }
    }

    fn gradient_step(&mut self, u_current: &mut [f64]) {
        assert_eq!(
            0,
            (self.problem.gradf)(u_current, &mut self.cache.work_gradient_u),
            "The computation of the gradient of the cost failed miserably"
        );
        // take a gradient step: u_currect -= gamma * gradient
        u_current
            .iter_mut()
            .zip(self.cache.work_gradient_u.iter())
            .for_each(|(u, w)| *u -= self.cache.gamma * *w);
    }

    fn projection_step(&mut self, u_current: &mut [f64]) {
        self.problem.constraints.project(u_current);
    }
}

/* --------------------------------------------------------------------------------------------- */

impl<'a, GradientType, ConstraintType, CostType> AlgorithmEngine
    for FBSEngine<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
{
    /// Take a forward-backward step and check whether the algorithm should terminate
    ///
    /// ## Arguments
    ///
    /// - `u_current` the current mutable
    ///
    /// ## Returns
    ///
    /// - A boolean flag which is`true` if and only if the algorith should not
    ///   terminate
    ///
    /// ## Panics
    ///
    /// The method may panick if the computation of the gradient of the cost function
    /// or the cost function panics.
    fn step(&mut self, u_current: &mut [f64]) -> bool {
        self.cache.work_u_previous.copy_from_slice(u_current); // cache the previous step
        self.gradient_step(u_current); // compute the gradient
        self.projection_step(u_current); // project
        self.cache.norm_fpr =
            matrix_operations::norm_inf_diff(u_current, &self.cache.work_u_previous);
        self.cache.norm_fpr > self.cache.tolerance
    }

    fn init(&mut self, _u_current: &mut [f64]) {}
}

/* --------------------------------------------------------------------------------------------- */

/// Optimiser using forward-backward splitting iterations (projected gradient)
///
/// Note that an `FBSOptimizer` holds a reference to an instance of `FBSEngine`
/// which needs to be created externally. A mutable reference to that `FBSEgnine`
/// is provided to the optimizer.
///
/// The `FBSEngine` is supposed to be updated whenever you need to solve
/// a different optimization problem.
///
///
pub struct FBSOptimizer<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
{
    fbs_engine: &'a mut FBSEngine<'a, GradientType, ConstraintType, CostType>,
    max_iter: usize,
}

impl<'a, GradientType, ConstraintType, CostType>
    FBSOptimizer<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
{
    pub fn new(
        fbs_engine: &'a mut FBSEngine<'a, GradientType, ConstraintType, CostType>,
    ) -> FBSOptimizer<'a, GradientType, ConstraintType, CostType> {
        FBSOptimizer {
            fbs_engine: fbs_engine,
            max_iter: MAX_ITER,
        }
    }

    /// Sets the tolerance
    pub fn with_tolerance(
        &mut self,
        tolerance: f64,
    ) -> &mut FBSOptimizer<'a, GradientType, ConstraintType, CostType> {
        assert!(tolerance > 0.0);
        self.fbs_engine.cache.tolerance = tolerance;
        self
    }

    /// Sets the maximum number of iterations
    pub fn with_max_iter(
        &mut self,
        max_iter: usize,
    ) -> &mut FBSOptimizer<'a, GradientType, ConstraintType, CostType> {
        self.max_iter = max_iter;
        self
    }
}

impl<'life, GradientType, ConstraintType, CostType> Optimizer
    for FBSOptimizer<'life, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'life,
    CostType: Fn(&[f64], &mut f64) -> i32,
    ConstraintType: constraints::Constraint + 'life,
{
    fn solve(&mut self, u: &mut [f64]) -> SolverStatus {
        self.fbs_engine.init(u);
        let mut num_iter: usize = 0;
        while self.fbs_engine.step(u) && num_iter < self.max_iter {
            num_iter += 1;
        }

        // cost at the solution
        let mut cost_value = 0.0;
        assert_eq!(
            0,
            (self.fbs_engine.problem.cost)(u, &mut cost_value),
            "The computation of the cost value at the solution failed"
        );
        // export solution status
        SolverStatus::new(
            num_iter < self.max_iter,
            num_iter,
            self.fbs_engine.cache.norm_fpr,
            cost_value,
        )
    }
}

/* --------------------------------------------------------------------------------------------- */

/// Cache for PANOC
pub struct PANOCCache {
    lbfgs: lbfgs::Estimator,
    gradient_u: Vec<f64>,
    u_half_step: Vec<f64>,
    gradient_step: Vec<f64>,
    direction_lbfgs: Vec<f64>,
    u_plus: Vec<f64>,
    rhs_ls: f64,
    lhs_ls: f64,
    fixed_point_residual: Vec<f64>,
    gamma: f64,
    tolerance: f64,
    norm_fpr: f64,
    tau: f64,
    lipschitz_constant: f64,
    sigma: f64,
    cost_value: f64,
}

impl PANOCCache {
    pub fn new(n: usize, tolerance: f64, lbfgs_mem: usize) -> PANOCCache {
        assert!(tolerance > 0., "tolerance must be positive");
        PANOCCache {
            gradient_u: vec![0.0; n],
            u_half_step: vec![0.0; n],
            fixed_point_residual: vec![0.0; n],
            direction_lbfgs: vec![0.0; n],
            gradient_step: vec![0.0; n],
            u_plus: vec![0.0; n],
            gamma: 0.0,
            tolerance: tolerance,
            norm_fpr: std::f64::INFINITY,
            lbfgs: lbfgs::Estimator::new(n, lbfgs_mem),
            lhs_ls: 0.0,
            rhs_ls: 0.0,
            tau: 1.0,
            lipschitz_constant: 0.0,
            sigma: 0.0,
            cost_value: 0.0,
        }
    }
}

/* --------------------------------------------------------------------------------------------- */

pub struct PANOCEngine<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
{
    problem: Problem<GradientType, ConstraintType, CostType>,
    cache: &'a mut PANOCCache,
}

impl<'a, GradientType, ConstraintType, CostType>
    PANOCEngine<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
{
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
        lipest.with_delta(1e-10).with_epsilon(1e-10);
        self.cache.lipschitz_constant = lipest.estimate_local_lipschitz();
    }

    fn gradient_step(&mut self, u_current: &[f64]) {
        // take a gradient step: gradient_step = u_current - gamma * gradient
        let gamma = self.cache.gamma;
        self.cache
            .gradient_step
            .iter_mut()
            .zip(u_current.iter())
            .zip(self.cache.gradient_u.iter())
            .for_each(|((grad_step, u), grad)| *grad_step = *u - gamma * *grad);
    }

    fn half_step(&mut self, u_current: &mut [f64]) {
        self.cache.u_half_step.copy_from_slice(u_current);
        self.problem
            .constraints
            .project(&mut self.cache.u_half_step);
        // u_half_step = projection(gradient_step)
    }

    fn update_lipschitz_constant(&mut self) {}

    fn line_search_condition(&mut self) -> bool {
        //let gamma = self.cache.gamma;
        //let tau = self.cache.tau;
        false
    }
}

impl<'a, GradientType, ConstraintType, CostType> AlgorithmEngine
    for PANOCEngine<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
{
    fn step(&mut self, u_current: &mut [f64]) -> bool {
        let gamma = self.cache.gamma;
        // compute the fixed point residual
        self.cache
            .fixed_point_residual
            .iter_mut()
            .zip(u_current.iter())
            .zip(self.cache.u_half_step.iter())
            .for_each(|((fpr, u), uhalf)| *fpr = (u - uhalf) / gamma);
        // compute the norm of FPR
        self.cache.norm_fpr = matrix_operations::norm2(&self.cache.fixed_point_residual);
        // exit if the norm of the fpr is adequetely small
        if self.cache.norm_fpr < self.cache.tolerance {
            return false;
        }
        // update the LBFGS buffer
        self.cache
            .lbfgs
            .update_hessian(&self.cache.gradient_u, u_current, 1.0, 1e-10);
        // compute an LBFGS direction
        self.cache
            .lbfgs
            .apply_hessian(&mut self.cache.direction_lbfgs);
        // compute dist squared
        let dist_squared = self
            .cache
            .gradient_step
            .iter()
            .zip(self.cache.u_half_step.iter())
            .map(|(g, uh)| (*g - *uh).powi(2))
            .sum::<f64>();
        // compute LHS
        self.cache.rhs_ls = self.cache.cost_value
            - (self.cache.gamma / 2.0) * matrix_operations::norm2(&self.cache.gradient_u)
            + dist_squared
            - self.cache.sigma
            + self.cache.norm_fpr.powi(2);

        self.cache.tau = 1.0;

        // perform line search

        false
    }

    fn init(&mut self, u_current: &mut [f64]) {
        self.estimate_loc_lip(u_current); // computes the gradient as well
        (self.problem.cost)(u_current, &mut self.cache.cost_value); // cost value
        self.cache.gamma = GAMMA_L_COEFF / self.cache.lipschitz_constant;
        self.cache.sigma = (1.0 - GAMMA_L_COEFF) * SIGMA_COEFF * self.cache.gamma;
        self.gradient_step(u_current);
        self.half_step(u_current);
    }
}

/* --------------------------------------------------------------------------------------------- */
/*          TESTS                                                                                */
/* --------------------------------------------------------------------------------------------- */
#[cfg(test)]
mod tests {

    use super::*;
    use crate::constraints;

    const N_DIM: usize = 2;

    fn my_cost(u: &[f64], cost: &mut f64) -> i32 {
        *cost = u[0] * u[0] + 2. * u[1] * u[1] + u[0] - u[1] + 3.0;
        0
    }

    fn my_gradient(u: &[f64], grad: &mut [f64]) -> i32 {
        grad[0] = u[0] + u[1] + 1.0;
        grad[1] = u[0] + 2. * u[1] - 1.0;
        0
    }

    #[test]
    fn fbs_step_no_constraints() {
        let no_constraints = constraints::NoConstraints::new();
        let problem = Problem::new(no_constraints, my_gradient, my_cost);
        let gamma = 0.1;
        let tolerance = 1e-6;
        let mut fbs_cache = FBSCache::new(N_DIM, gamma, tolerance);
        {
            let mut fbs_engine = FBSEngine::new(problem, &mut fbs_cache);
            let mut u = [1.0, 3.0];
            assert_eq!(true, fbs_engine.step(&mut u));
            assert_eq!([0.5, 2.4], u);
        }
        assert_eq!([1., 3.], *fbs_cache.work_u_previous);
    }

    #[test]
    fn fbs_step_ball_constraints() {
        let no_constraints = constraints::Ball2::new_at_origin_with_radius(0.1);
        let problem = Problem::new(no_constraints, my_gradient, my_cost);
        let gamma = 0.1;
        let tolerance = 1e-6;
        let mut fbs_cache = FBSCache::new(N_DIM, gamma, tolerance);
        let mut fbs_engine = FBSEngine::new(problem, &mut fbs_cache);

        let mut u = [1.0, 3.0];

        assert_eq!(true, fbs_engine.step(&mut u));
        assert!((u[0] - 0.020395425411200).abs() < 1e-14);
        assert!((u[1] - 0.097898041973761).abs() < 1e-14);
    }

    #[test]
    fn solve_fbs() {
        let radius = 0.2;
        let box_constraints = constraints::Ball2::new_at_origin_with_radius(radius);
        let problem = Problem::new(box_constraints, my_gradient, my_cost);
        let gamma = 0.1;
        let tolerance = 1e-6;
        let mut fbs_cache = FBSCache::new(N_DIM, gamma, tolerance);
        let mut fbs_engine = FBSEngine::new(problem, &mut fbs_cache);
        let mut u = [0.0; N_DIM];
        let mut optimizer = FBSOptimizer::new(&mut fbs_engine);
        let status = optimizer.solve(&mut u);
        assert!(status.has_converged());
        assert!(status.get_norm_fpr() < tolerance);
        assert!(status.get_number_iterations() <= MAX_ITER);
        assert!((-0.14896 - u[0]).abs() < 1e-4);
        assert!((0.13346 - u[1]).abs() < 1e-4);
    }

    #[test]
    fn solve_fbs_many_times() {
        // Algorithm configuration
        let gamma = 0.1;
        let tolerance = 1e-6;

        // The cache is constructed ONCE. This step allocates memory.
        let mut fbs_cache = FBSCache::new(N_DIM, gamma, tolerance);

        let mut u = [0.0; 2];

        for _i in 1..10 {
            // Every time NMPC is executed, the constraints may change
            let box_constraints = constraints::Ball2::new_at_origin_with_radius(0.2);
            // The problem is surely update at every execution of NMPC
            let problem = Problem::new(box_constraints, my_gradient, my_cost);
            // Construct a new Engine; this does not allocate any memory
            let mut fbs_engine = FBSEngine::new(problem, &mut fbs_cache);
            // Here comes the new initial condition
            u[0] = 2.0 * _i as f64;
            u[1] = -_i as f64;
            // Create a new optimizer...
            let mut optimizer = FBSOptimizer::new(&mut fbs_engine);
            let status = optimizer.solve(&mut u);
            assert!(status.get_norm_fpr() < tolerance);
        }
    }

    #[test]
    fn make_panoc_init() {
        let radius = 0.2;
        let box_constraints = constraints::Ball2::new_at_origin_with_radius(radius);
        let problem = Problem::new(box_constraints, my_gradient, my_cost);
        let mut panoc_cache = PANOCCache::new(N_DIM, 1e-6, 5);
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);
        let mut u = [0.75, -1.4];
        panoc_engine.init(&mut u);
        println!("Lip = {}", panoc_engine.cache.lipschitz_constant);
        println!("gamma = {}", panoc_engine.cache.gamma);
        println!("sigma = {}", panoc_engine.cache.sigma);

        panoc_engine.step(&mut u);
    }

}
