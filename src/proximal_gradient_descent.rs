use crate::constraints::Constraint;

pub struct ProjectedGradient<'life, GradientType, ConstraintType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    ConstraintType: Constraint,
{
    grad: GradientType,
    projection: ConstraintType,
    gradient_workspace: &'life mut [f64],
}

impl<'life, GradientType, ConstraintType> ProjectedGradient<'life, GradientType, ConstraintType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    ConstraintType: Constraint,
{
    pub fn new(
        gradient_function: GradientType,
        constraints_set: ConstraintType,
        gradient: &'life mut [f64],
    ) -> ProjectedGradient<GradientType, ConstraintType> {
        ProjectedGradient {
            grad: gradient_function,
            projection: constraints_set,
            gradient_workspace: gradient,
        }
    }

    pub fn gradient_step(&mut self, u: &mut [f64], gamma: f64) {
        (self.grad)(u, self.gradient_workspace);
        u.iter_mut()
            .zip(self.gradient_workspace.iter())
            .for_each(|(u, w)| *u -= gamma * *w);
    }

    pub fn projected_gradient_step(&mut self, u: &mut [f64], gamma: f64) {
        self.gradient_step(u, gamma);
        self.projection.project(u);
    }

    pub fn get_gradient(&self) -> &[f64] {
        self.gradient_workspace
    }
}

#[cfg(test)]
mod tests {

    use super::ProjectedGradient;
    use crate::constraints;
    use crate::matrix_operations;

    fn my_gradient(u: &[f64], grad: &mut [f64]) -> i32 {
        grad[0] = 3.0 * u[0] + u[1] - 1.0;
        grad[1] = -2.0 * u[0] + 2.0;
        0
    }

    #[test]
    fn gradient_step() {
        let mut gradient_workspace = vec![0.0; 2];
        let xmin = vec![0.0, -2.0];
        let xmax = vec![1.0, 5.4];
        let box_constraints = constraints::Rectangle::new(xmin, xmax);
        let mut prox_grad_method =
            ProjectedGradient::new(my_gradient, box_constraints, &mut gradient_workspace);
        let mut u = [1.0, -0.5];
        let gamma = 0.15;
        prox_grad_method.gradient_step(&mut u, gamma);

        let u_expected = [0.775, -0.5];
        assert_eq!(u_expected, u);
    }

    #[test]
    fn projected_gradient_step() {
        let mut gradient_workspace = vec![0.0; 2];
        let ball = constraints::Ball2::new_at_origin_with_radius(0.5);
        let mut prox_grad_method =
            ProjectedGradient::new(my_gradient, ball, &mut gradient_workspace);
        let mut u = [1.0, -0.5];
        let gamma = 0.15;
        prox_grad_method.projected_gradient_step(&mut u, gamma);
        assert_eq!(0.5, matrix_operations::norm2(&u));
        let u_expected = [0.42014832411212216, -0.27106343491104656];
        assert_eq!(u_expected, u);
    }

    #[test]
    fn solve_projected_gradient() {
        let mut gradient_workspace = vec![0.0; 2];
        let ball = constraints::Ball2::new_at_origin_with_radius(0.2);
        let mut prox_grad_method =
            ProjectedGradient::new(my_gradient, ball, &mut gradient_workspace);
        let mut u = [1.0, -0.5];
        let mut u_previous = u.clone();

        let gamma = 0.02;
        let mut norm_fpr = 1.0;
        let max_iter = 20;
        let epsilon = 1e-3;
        let mut num_iter = 0_usize;

        while norm_fpr > epsilon && num_iter < max_iter {
            prox_grad_method.projected_gradient_step(&mut u, gamma);
            norm_fpr = matrix_operations::norm_inf_diff(&u, &u_previous);
            u_previous.copy_from_slice(&u); // u_previous = u
            num_iter += 1;
        }
        assert!(num_iter < max_iter);
    }

}
