pub struct ProjectedGradient<'life, Gradient, Set>
where
    Gradient: Fn(&[f64], &mut [f64]) -> i32,
    Set: crate::constraints::Constraint,
{
    grad: Gradient,
    projection: Set,
    gradient_workspace: &'life mut [f64],
}

impl<'life, Gradient, Set> ProjectedGradient<'life, Gradient, Set>
where
    Gradient: Fn(&[f64], &mut [f64]) -> i32,
    Set: crate::constraints::Constraint,
{
    pub fn new(
        gradient_: Gradient,
        constraints_set_: Set,
        gradient: &'life mut [f64],
    ) -> ProjectedGradient<Gradient, Set> {
        ProjectedGradient {
            grad: gradient_,
            projection: constraints_set_,
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

    pub fn norm_grad(&self) -> f64 {
        crate::matrix_operations::norm_inf(self.gradient_workspace)
    }
}

#[cfg(test)]
mod tests {

    fn my_gradient(u: &[f64], grad: &mut [f64]) -> i32 {
        grad[0] = 3. * u[0] + u[1] - 1.0;
        grad[1] = -7.0 * u[0] + 2.;
        0
    }
    #[test]
    fn test0() {
        let mut gradient_ws = vec![0.0; 2];
        let ball = crate::constraints::Ball2::new_at_origin_with_radius(1.0);
        let mut prox_grad = super::ProjectedGradient::new(my_gradient, ball, &mut gradient_ws);
        let mut u = vec![0.0; 2];
        let gamma = 1e-1;
        for _i in 1..100 {
            prox_grad.projected_gradient_step(&mut u, gamma);
            let norm_gradient = prox_grad.norm_grad();
            println!("norm = {}", norm_gradient);
        }

        println!("u = {:?}", u);
    }
}
