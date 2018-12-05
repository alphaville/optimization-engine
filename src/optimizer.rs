use crate::constraints;
use crate::matrix_operations;
use crate::proximal_gradient_descent::ProjectedGradient;

static MAX_ITER: usize = 100_usize;
static TOLERANCE: f64 = 1e-4;

pub struct SolverStatus {
    pub converged: bool,
    pub num_iter: usize,
    pub fpr_norm: f64,
}

impl SolverStatus {
    pub fn new(converged: bool, num_iter: usize, fpr_norm: f64) -> SolverStatus {
        SolverStatus {
            converged: converged,
            num_iter: num_iter,
            fpr_norm: fpr_norm,
        }
    }
}

pub struct Optimizer<GradientType, ConstraintType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    ConstraintType: constraints::Constraint,
{
    constraints: ConstraintType,
    gradf: GradientType,
    work_gradient_u: Vec<f64>,
    work_u_previous: Vec<f64>,
    gamma: f64,
    max_iter: usize,
    epsilon: f64,
}

impl<GradientType, ConstraintType> Optimizer<GradientType, ConstraintType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    ConstraintType: constraints::Constraint,
{
    pub fn new(
        c_: ConstraintType,
        g_: GradientType,
        len: usize,
    ) -> Optimizer<GradientType, ConstraintType> {
        Optimizer {
            constraints: c_,
            gradf: g_,
            work_gradient_u: vec![0.0; len],
            work_u_previous: vec![0.0; len],
            gamma: 0.1_f64,
            max_iter: MAX_ITER,
            epsilon: TOLERANCE,
        }
    }

    pub fn with_epsilon(&mut self, epsilon: f64) -> &mut Optimizer<GradientType, ConstraintType> {
        assert!(epsilon > 0.0);
        self.epsilon = epsilon;
        self
    }

    pub fn with_gamma(&mut self, gamma: f64) -> &mut Optimizer<GradientType, ConstraintType> {
        assert!(gamma > 0.0);
        self.gamma = gamma;
        self
    }

    pub fn with_max_iter(
        &mut self,
        max_iter: usize,
    ) -> &mut Optimizer<GradientType, ConstraintType> {
        self.max_iter = max_iter;
        self
    }

    pub fn solve(&mut self, u: &mut [f64]) -> SolverStatus {
        let mut pg_method =
            ProjectedGradient::new(&self.gradf, &self.constraints, &mut self.work_gradient_u);
        self.work_u_previous.copy_from_slice(u);
        let mut norm_fpr = std::f64::INFINITY;
        let mut num_iter = 0_usize;
        while norm_fpr > self.epsilon && num_iter < self.max_iter {
            pg_method.projected_gradient_step(u, self.gamma);
            norm_fpr = matrix_operations::norm_inf_diff(&u, &self.work_u_previous);
            self.work_u_previous.copy_from_slice(&u);
            num_iter += 1;
        }
        SolverStatus::new(norm_fpr < self.epsilon, num_iter, norm_fpr)
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::constraints;

    fn my_gradient(u: &[f64], grad: &mut [f64]) -> i32 {
        grad[0] = 3.0 * u[0] + u[1] - 1.0;
        grad[1] = -2.0 * u[0] + 2.0;
        0
    }

    #[test]
    fn solve_problem() {
        let box_constraints = constraints::Ball2::new_at_origin_with_radius(0.2);
        let mut problem = Optimizer::new(box_constraints, my_gradient, 2);
        problem
            .with_epsilon(1e-4)
            .with_gamma(0.1)
            .with_max_iter(100);
        let mut u_init = [1.0, -0.5];
        let status = problem.solve(&mut u_init);
        assert!(status.converged);
        println!("log |fpr| = {:.2}", status.fpr_norm.log10());
    }

}
