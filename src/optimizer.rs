//! Optimisation algorithms
//!
//!
use crate::constraints;
use crate::matrix_operations;

// default maximum number of iterations
static MAX_ITER: usize = 100_usize;

/* ---------------------------------------------------------------------------- */

/// Solver status
pub struct SolverStatus {
    /// whether the algorithm has converged
    converged: bool,
    /// number of iterations for convergence
    num_iter: usize,
    /// norm of the fixed-point residual (FPR)
    fpr_norm: f64,
    /// cost value
    cost_value: f64,
}

impl SolverStatus {
    /// Constructs a new instance of SolverStatus
    pub fn new(converged: bool, num_iter: usize, fpr_norm: f64, cost_value: f64) -> SolverStatus {
        SolverStatus {
            converged: converged,
            num_iter: num_iter,
            fpr_norm: fpr_norm,
            cost_value: cost_value,
        }
    }

    pub fn has_converged(&self) -> bool {
        self.converged
    }

    pub fn get_number_iterations(&self) -> usize {
        self.num_iter
    }

    pub fn get_norm_fpr(&self) -> f64 {
        self.fpr_norm
    }

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

/// A step of an algorithm
pub trait AlgorithmStep {
    /// Take a step of the algorithm and return `true` only if the iterations should continue
    fn step(&mut self, &mut [f64]) -> bool;
}

/* ---------------------------------------------------------------------------- */

/// Definition of an optimisation problem
pub struct Problem<GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    CostType: Fn(&[f64], &mut f64) -> i32,
    ConstraintType: constraints::Constraint,
{
    /// constraints
    pub constraints: ConstraintType,
    /// gradient of the cost
    pub gradf: GradientType,
    pub cost: CostType,
    /// dimension of decision variable
    pub n: usize,
    //TODO Add cost function itself
}

impl<GradientType, ConstraintType, CostType> Problem<GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    CostType: Fn(&[f64], &mut f64) -> i32,
    ConstraintType: constraints::Constraint,
{
    /// Construct a new instance of an optimisation problem
    pub fn new(
        cnstr_: ConstraintType,
        gradf_: GradientType,
        ell_: CostType,
        len: usize,
    ) -> Problem<GradientType, ConstraintType, CostType> {
        Problem {
            constraints: cnstr_,
            gradf: gradf_,
            n: len,
            cost: ell_,
        }
    }
}

/* ---------------------------------------------------------------------------- */

struct FBSStep<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
{
    problem: &'a Problem<GradientType, ConstraintType, CostType>,
    work_gradient_u: Vec<f64>,
    work_u_previous: Vec<f64>,
    gamma: f64,
    tolerance: f64,
    norm_fpr: f64,
}

impl<'a, GradientType, ConstraintType, CostType> FBSStep<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
{
    pub fn new(
        problem: &'a Problem<GradientType, ConstraintType, CostType>,
        gamma: f64,
        tolerance: f64,
    ) -> FBSStep<'a, GradientType, ConstraintType, CostType> {
        FBSStep {
            problem: problem,
            work_gradient_u: vec![0.0; problem.n],
            work_u_previous: vec![0.0; problem.n],
            gamma: gamma,
            tolerance: tolerance,
            norm_fpr: std::f64::INFINITY,
        }
    }

    fn gradient_step(&mut self, u_current: &mut [f64]) {
        (self.problem.gradf)(u_current, &mut self.work_gradient_u);
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

impl<'a, GradientType, ConstraintType, CostType> AlgorithmStep
    for FBSStep<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
{
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
pub struct FBSOptimizer<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
{
    fbs_step: FBSStep<'a, GradientType, ConstraintType, CostType>,
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
        problem: &'a Problem<GradientType, ConstraintType, CostType>,
        gamma: f64,
        epsilon: f64,
    ) -> FBSOptimizer<GradientType, ConstraintType, CostType> {
        FBSOptimizer {
            fbs_step: FBSStep::new(problem, gamma, epsilon),
            max_iter: MAX_ITER,
        }
    }

    /// Sets the tolerance
    pub fn with_epsilon(
        &mut self,
        epsilon: f64,
    ) -> &mut FBSOptimizer<'a, GradientType, ConstraintType, CostType> {
        assert!(epsilon > 0.0);
        self.fbs_step.tolerance = epsilon;
        self
    }

    /// Sets parameter gamma
    pub fn with_gamma(
        &mut self,
        gamma: f64,
    ) -> &mut FBSOptimizer<'a, GradientType, ConstraintType, CostType> {
        assert!(gamma > 0.0);
        self.fbs_step.gamma = gamma;
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

        while self.fbs_step.step(u) && num_iter < self.max_iter {
            num_iter += 1;
        }

        // cost at the solution
        let mut cost_value = 0.0;
        (self.fbs_step.problem.cost)(u, &mut cost_value);

        // export solution status
        SolverStatus::new(
            num_iter < self.max_iter,
            num_iter,
            self.fbs_step.norm_fpr,
            cost_value,
        )
    }
}

/* ---------------------------------------------------------------------------- */

struct PANOCStep<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
{
    problem: &'a Problem<GradientType, ConstraintType, CostType>,
    work_gradient_u: Vec<f64>,
    lbfgs: lbfgs::Estimator,
    work_u_previous: Vec<f64>,
    gamma: f64,
    tolerance: f64,
    norm_fpr: f64,
}

impl<'a, GradientType, ConstraintType, CostType>
    PANOCStep<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
{
    pub fn new(
        problem: &'a Problem<GradientType, ConstraintType, CostType>,
        gamma: f64,
        tolerance: f64,
        mem: usize,
    ) -> PANOCStep<'a, GradientType, ConstraintType, CostType> {
        PANOCStep {
            problem: problem,
            lbfgs: lbfgs::Estimator::new(problem.n, mem),
            work_gradient_u: vec![0.0; problem.n],
            work_u_previous: vec![0.0; problem.n],
            gamma: gamma,
            tolerance: tolerance,
            norm_fpr: std::f64::INFINITY,
        }
    }
}

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
#[cfg(test)]
mod tests {

    use super::*;
    use crate::constraints;

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
        let problem = Problem::new(no_constraints, my_gradient, my_cost, 2);
        let gamma = 0.1;
        let tolerance = 1e-6;
        let mut fbs_step = FBSStep::new(&problem, gamma, tolerance);
        let mut u = [1.0, 3.0];
        assert_eq!(true, fbs_step.step(&mut u));
        assert_eq!([0.5, 2.4], u);
        assert_eq!([1., 3.], *fbs_step.work_u_previous);
    }

    #[test]
    fn fbs_step_ball_constraints() {
        let no_constraints = constraints::Ball2::new_at_origin_with_radius(0.1);
        let problem = Problem::new(no_constraints, my_gradient, my_cost, 2);
        let gamma = 0.1;
        let tolerance = 1e-6;
        let mut fbs_step = FBSStep::new(&problem, gamma, tolerance);

        let mut u = [1.0, 3.0];

        assert_eq!(true, fbs_step.step(&mut u));
        assert!((u[0] - 0.020395425411200).abs() < 1e-14);
        assert!((u[1] - 0.097898041973761).abs() < 1e-14);
    }

    #[test]
    fn solve_problem() {
        // Define constraints
        let radius = 0.2;
        let box_constraints = constraints::Ball2::new_at_origin_with_radius(radius);

        // Define problem
        let problem = Problem::new(box_constraints, my_gradient, my_cost, 2);

        // Solver parameters
        let gamma = 0.1;
        let tolerance = 1e-6;
        let max_iter = 100;

        // Construct optimizer
        let mut optimizer = FBSOptimizer::new(&problem, gamma, tolerance);
        optimizer.with_max_iter(max_iter);

        // Solve
        let mut u = [0.0; 2];
        let status = optimizer.solve(&mut u);

        // Check solution status
        assert!(status.has_converged());
        assert!(status.get_norm_fpr() < tolerance);
        assert!(status.get_number_iterations() <= max_iter);

        // Solve again starting at the solution
        let status = optimizer.solve(&mut u);
        assert_eq!(0, status.get_number_iterations());
        assert!(matrix_operations::norm2(&u) <= radius); // check feasibility
    }

    #[test]
    fn make_panoc() {
        let box_constraints = constraints::Ball2::new_at_origin_with_radius(0.2);
        let problem = Problem::new(box_constraints, my_gradient, my_cost, 2);

        let gamma = 0.1;
        let tolerance = 1e-6;
        let mem = 5;
        let mut panoc_step = PANOCStep::new(&problem, gamma, tolerance, mem);

        println!(
            "{:?}",
            panoc_step
                .lbfgs
                .update_hessian(&[1., 1.], &[2., 3.], 1., 1e-6)
        );

        println!(
            "{:?}",
            panoc_step
                .lbfgs
                .update_hessian(&[1.01, 1.1], &[2., 3.05], 1., 1e-6)
        );

        let mut p = [1.1, 0.4];
        panoc_step.lbfgs.apply_hessian(&mut p);
        println!("p = {:?}", p);
    }

}
