use crate::constraints;
use crate::matrix_operations;
use crate::proximal_gradient_descent::ProjectedGradient;

// default maximum number of iterations
static MAX_ITER: usize = 100_usize;

/* ---------------------------------------------------------------------------- */

/// Solver status
pub struct SolverStatus {
    /// whether the algorithm has converged
    pub converged: bool,
    /// number of iterations for convergence
    pub num_iter: usize,
    /// norm of the fixed-point residual (FPR)
    pub fpr_norm: f64,
    /// cost value
    pub cost_value: f64,
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

pub trait AlgorithmStep {
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
    pub problem: &'a Problem<GradientType, ConstraintType, CostType>,
    pub work_gradient_u: Vec<f64>,
    pub work_u_previous: Vec<f64>,
    pub gamma: f64,
    pub tolerance: f64,
    pub norm_fpr: f64,
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

// struct PANOCCache {
//     work_gradient_u: Vec<f64>,
//     work_u_previous: Vec<f64>,
//     lbfgs: lbfgs::Estimator,
// }

// impl PANOCCache {
//     fn new(n: usize, mem: usize) -> PANOCCache {
//         PANOCCache {
//             work_gradient_u: vec![0.0; n],
//             work_u_previous: vec![0.0; n],
//             lbfgs: lbfgs::Estimator::new(n, mem),
//         }
//     }
// }

/* ---------------------------------------------------------------------------- */

/// Solves a given problem using PANOC
// pub struct PANOCOptimizer<'life, GradientType, ConstraintType, CostType>
// where
//     GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'life,
//     CostType: Fn(&[f64], &mut f64) -> i32 + 'life,
//     ConstraintType: constraints::Constraint + 'life,
// {
//     problem: &'life Problem<GradientType, ConstraintType, CostType>,
//     cache: PANOCCache,
// }

// impl<'life, GradientType, ConstraintType, CostType>
//     PANOCOptimizer<'life, GradientType, ConstraintType, CostType>
// where
//     GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'life,
//     CostType: Fn(&[f64], &mut f64) -> i32 + 'life,
//     ConstraintType: constraints::Constraint + 'life,
// {
//     pub fn new(
//         problem: &'life Problem<GradientType, ConstraintType, CostType>,
//         mem: usize,
//     ) -> PANOCOptimizer<'life, GradientType, ConstraintType, CostType> {
//         PANOCOptimizer {
//             problem: problem,
//             cache: PANOCCache::new(problem.n, mem),
//         }
//     }
// }

// impl<'life, GradientType, ConstraintType, CostType> Optimizer
//     for PANOCOptimizer<'life, GradientType, ConstraintType, CostType>
// where
//     GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'life,
//     CostType: Fn(&[f64], &mut f64) -> i32 + 'life,
//     ConstraintType: constraints::Constraint + 'life,
// {
//     fn solve(&mut self, u: &mut [f64]) -> SolverStatus {
//         let mut lip_estimator = lipschitz_estimator::LipschitzEstimator::new(
//             u,
//             &self.problem.gradf,
//             &mut self.cache.work_gradient_u,
//         );
//         let lipshitz_constant = lip_estimator.estimate_local_lipschitz();
//         SolverStatus::new(true, 1, lipshitz_constant, 0.0)
//     }
// }


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
        let box_constraints = constraints::Ball2::new_at_origin_with_radius(0.2);
        let problem = Problem::new(box_constraints, my_gradient, my_cost, 2);
        let gamma = 0.1;
        let tolerance = 1e-6;
        let mut optimizer = FBSOptimizer::new(&problem, gamma, tolerance);
        optimizer.with_max_iter(10000);
        let mut u = [0.0; 2];
        let status = optimizer.solve(&mut u);
        assert!(status.converged);
        let status = optimizer.solve(&mut u);
        assert_eq!(0, status.num_iter);
        assert!(matrix_operations::norm2(&u) <= 0.2);
    }

    // #[test]
    // fn make_panoc() {
    //     let box_constraints = constraints::Ball2::new_at_origin_with_radius(0.2);
    //     let problem = Problem::new(box_constraints, my_gradient, my_cost, 2);
    //     let mut panoc = PANOCOptimizer::new(&problem, 5);
    //     let mut u = [0.0; 2];
    //     panoc.solve(&mut u);
    // }

}
