//! Optimisation algorithms
//!
//!
use crate::constraints;
use crate::matrix_operations;

// default maximum number of iterations
const MAX_ITER: usize = 100_usize;

/* ---------------------------------------------------------------------------- */

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

/* ---------------------------------------------------------------------------- */

/// A general optimizer
pub trait Optimizer {
    /// solves a given problem and updates the initial estimate `u` with the solution
    ///
    /// Returns the solver status
    fn solve(&mut self, u: &mut [f64]) -> SolverStatus;
}

/* ---------------------------------------------------------------------------- */

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
}

/* ---------------------------------------------------------------------------- */

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
    /// dimension of decision variable
    n: usize,
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
    /// - `n` dimension of the vector of decision variables
    pub fn new(
        constraints: ConstraintType,
        cost_gradient: GradientType,
        cost: CostType,
        n: usize,
    ) -> Problem<GradientType, ConstraintType, CostType> {
        Problem {
            constraints: constraints,
            gradf: cost_gradient,
            n: n,
            cost: cost,
        }
    }
}

/* ---------------------------------------------------------------------------- */

/// Engine for the forward-backward splitting (FBS), or projected gradient, algorithm
///
///
pub struct FBSEngine<GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    CostType: Fn(&[f64], &mut f64) -> i32,
    ConstraintType: constraints::Constraint,
{
    problem: Problem<GradientType, ConstraintType, CostType>,
    work_gradient_u: Vec<f64>,
    work_u_previous: Vec<f64>,
    gamma: f64,
    tolerance: f64,
    norm_fpr: f64,
}

impl<GradientType, ConstraintType, CostType> FBSEngine<GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    CostType: Fn(&[f64], &mut f64) -> i32,
    ConstraintType: constraints::Constraint,
{
    /// Construct a new instance of `FBSEngine`
    ///
    /// ## Arguments
    ///
    /// - `problem` the definition of an optimization problem; note that the ownership
    ///   of `problem` is transferred to this method
    /// - `gamma` parameter gamma of the algorithm
    /// - `tolerance` tolerance used for termination; the algorithm terminates if the
    ///   infinity norm of the fixed-point residual is below `tolerance`
    ///
    /// ## Memory allocation
    ///
    /// This method allocates new memory (which it owns, of course). You should avoid
    /// constructing instances of `FBSEngine`  in a loop or in any way more than
    /// absolutely necessary
    ///
    /// If you need to call an optimizer more than once, perhaps with different
    /// parameters, then construct an `FBSEngine` only once
    ///
    pub fn new(
        problem: Problem<GradientType, ConstraintType, CostType>,
        gamma: f64,
        tolerance: f64,
    ) -> FBSEngine<GradientType, ConstraintType, CostType> {
        let n = problem.n;
        FBSEngine {
            problem: problem,
            work_gradient_u: vec![0.0; n],
            work_u_previous: vec![0.0; n],
            gamma: gamma,
            tolerance: tolerance,
            norm_fpr: std::f64::INFINITY,
        }
    }

    fn gradient_step(&mut self, u_current: &mut [f64]) {
        assert_eq!(
            0,
            (self.problem.gradf)(u_current, &mut self.work_gradient_u),
            "The computation of the gradient of the cost failed miserably"
        );
        // take a gradient step: u_currect -= gamma * gradient
        u_current
            .iter_mut()
            .zip(self.work_gradient_u.iter())
            .for_each(|(u, w)| *u -= self.gamma * *w);
    }

    fn projection_step(&mut self, u_current: &mut [f64]) {
        self.problem.constraints.project(u_current);
    }
}

impl<'a, GradientType, ConstraintType, CostType> AlgorithmEngine
    for FBSEngine<GradientType, ConstraintType, CostType>
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
        self.work_u_previous.copy_from_slice(u_current); // cache the previous step
        self.gradient_step(u_current); // compute the gradient
        self.projection_step(u_current); // project
        self.norm_fpr = matrix_operations::norm_inf_diff(u_current, &self.work_u_previous);
        self.norm_fpr > self.tolerance
    }
}

/* ---------------------------------------------------------------------------- */

/// Optimiser using forward-backward splitting iterations (projected gradient)
///
/// Note that an `FBSOptimizer` holds a reference to an instance of `FBSEngine`
/// which needs to be created externally. A mutable reference to that `FBSEgnine`
/// is provided to the optimizer.
///
pub struct FBSOptimizer<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
{
    fbs_engine: &'a mut FBSEngine<GradientType, ConstraintType, CostType>,
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
        fbs_engine: &'a mut FBSEngine<GradientType, ConstraintType, CostType>,
    ) -> FBSOptimizer<'a, GradientType, ConstraintType, CostType> {
        FBSOptimizer {
            fbs_engine: fbs_engine,
            max_iter: MAX_ITER,
        }
    }

    /// Sets the tolerance
    pub fn with_epsilon(
        &mut self,
        tolerance: f64,
    ) -> &mut FBSOptimizer<'a, GradientType, ConstraintType, CostType> {
        assert!(tolerance > 0.0);
        self.fbs_engine.tolerance = tolerance;
        self
    }

    /// Sets parameter gamma
    pub fn with_gamma(
        &mut self,
        gamma: f64,
    ) -> &mut FBSOptimizer<'a, GradientType, ConstraintType, CostType> {
        assert!(gamma > 0.0);
        self.fbs_engine.gamma = gamma;
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
            self.fbs_engine.norm_fpr,
            cost_value,
        )
    }
}

/* ---------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
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
        let problem = Problem::new(no_constraints, my_gradient, my_cost, N_DIM);
        let gamma = 0.1;
        let tolerance = 1e-6;
        let mut fbs_step = FBSEngine::new(problem, gamma, tolerance);
        let mut u = [1.0, 3.0];
        assert_eq!(true, fbs_step.step(&mut u));
        assert_eq!([0.5, 2.4], u);
        assert_eq!([1., 3.], *fbs_step.work_u_previous);
    }

    #[test]
    fn fbs_step_ball_constraints() {
        let no_constraints = constraints::Ball2::new_at_origin_with_radius(0.1);
        let problem = Problem::new(no_constraints, my_gradient, my_cost, N_DIM);
        let gamma = 0.1;
        let tolerance = 1e-6;
        let mut fbs_step = FBSEngine::new(problem, gamma, tolerance);

        let mut u = [1.0, 3.0];

        assert_eq!(true, fbs_step.step(&mut u));
        assert!((u[0] - 0.020395425411200).abs() < 1e-14);
        assert!((u[1] - 0.097898041973761).abs() < 1e-14);
    }

    #[test]
    fn solve_fbs() {
        let radius = 0.2;
        let box_constraints = constraints::Ball2::new_at_origin_with_radius(radius);
        let problem = Problem::new(box_constraints, my_gradient, my_cost, N_DIM);
        let gamma = 0.1;
        let tolerance = 1e-6;
        let mut fbs_step = FBSEngine::new(problem, gamma, tolerance);
        let mut u = [0.0; N_DIM];
        let mut optimizer = FBSOptimizer::new(&mut fbs_step);
        let status = optimizer.solve(&mut u);
        assert!(status.has_converged());
        assert!(status.get_norm_fpr() < tolerance);
        assert!(status.get_number_iterations() <= MAX_ITER);
        assert!((-0.14896 - u[0]).abs() < 1e-4);
        assert!((0.13346 - u[1]).abs() < 1e-4);
    }

    #[test]
    fn solve_fbs_many_times() {
        let radius = 0.2;
        let box_constraints = constraints::Ball2::new_at_origin_with_radius(radius);
        let problem = Problem::new(box_constraints, my_gradient, my_cost, 2);
        let gamma = 0.1;
        let tolerance = 1e-6;
        let max_iter = 100;
        let mut fbs_step = FBSEngine::new(problem, gamma, tolerance);
        let mut u = [0.0; 2];

        for _i in 1..10 {
            u[0] = 2.0 * _i as f64;
            u[1] = -_i as f64;
            let mut optimizer = FBSOptimizer::new(&mut fbs_step);
            optimizer.with_max_iter(max_iter);
            let status = optimizer.solve(&mut u);
            assert!(status.get_norm_fpr() < tolerance);
        }
    }

}
