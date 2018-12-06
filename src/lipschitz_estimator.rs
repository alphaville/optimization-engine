//!
//! Estimates a local Lipschitz constant for a given function `F: R^n -> R^n`
//!
//! Functions are provided at closures.
//!
//! # Method
//!
//! This function computes a numerical approximation of the norm of the directional
//! derivative of a function `F` along a direction `h = max {delta, epsilon*u}`, where `delta`
//! and `epsilon` are small numbers.
//!

pub struct LipschitzEstimator<'a, F>
where
    F: Fn(&[f64], &mut [f64]) -> i32 + 'a,
{
    /// `u_decision_var` is the point where the Lipschitz constant is estimated
    u_decision_var: &'a mut [f64],
    ///  internally allocated workspace memory
    workspace: Vec<f64>,
    /// `function_value_at_u` a vector which is updated with the
    /// value of the given function, `F`, at `u`; the provided value
    /// of `function_value_at_u_p` is not used
    function_value_at_u: &'a mut [f64],
    ///
    /// Function whose Lipschitz constant is to be approximated
    ///
    /// For example, in optimization, this is the gradient (Jacobian matrix)
    /// of the cost function (this is a closure)
    function: &'a F,
    epsilon_lip: f64,
    delta_lip: f64,
}

#[inline(always)]
fn norm2(a: &[f64]) -> f64 {
    a.iter()
        .map(|&x: &f64| -> f64 { x * x })
        .sum::<f64>()
        .sqrt()
}

impl<'a, F> LipschitzEstimator<'a, F>
where
    F: Fn(&[f64], &mut [f64]) -> i32 + 'a,
{
    /// Creates a new instance of this structure
    ///
    /// Input arguments:
    ///
    /// - `u_` On entry: point where the Lipschitz constant is estimated,
    ///    On exit: the provided slice is modified (this is why it is a mutable
    ///    reference). The value of `u_` at exit is slightly perturbed. If you need
    ///    to keep the original value of `u_`, you need to make a copy of the variable
    ///    before you provide it to this function.
    /// - `f_` given closure
    /// - `function_value_` externally allocated memory which on exit stores the
    ///    value of the given function at `u_`, that is `f_(u_)`

    pub fn new(
        u_: &'a mut [f64],
        f_: &'a F,
        function_value_: &'a mut [f64],
    ) -> LipschitzEstimator<'a, F> {
        let n: usize = u_.len();
        LipschitzEstimator {
            u_decision_var: u_,
            workspace: vec![0.0_f64; n],
            function_value_at_u: function_value_,
            function: f_,
            epsilon_lip: 1e-6,
            delta_lip: 1e-6,
        }
    }

    ///
    /// A setter method for `delta`
    ///
    /// # Panics
    /// The function will panic if `delta` is non positive
    ///
    pub fn with_delta(&mut self, delta: f64) -> &mut LipschitzEstimator<'a, F> {
        assert!(delta > 0.0);
        self.delta_lip = delta;
        self
    }

    ///
    /// A setter method for `epsilon`
    ///
    /// # Panics
    /// The function will panic if `epsilon` is non positive
    ///
    pub fn with_epsilon(&mut self, epsilon: f64) -> &mut LipschitzEstimator<'a, F> {
        assert!(epsilon > 0.0);
        self.epsilon_lip = epsilon;
        self
    }
    ///
    /// Getter method for the Jacobian
    ///
    /// During the computation of the local lipschitz constant at `u`,
    /// the value of the given function at `u` is computed and stored
    /// internally. This method returns a pointer to that vector.
    ///
    /// If `estimate_local_lipschitz` has not been computed, the result
    /// will point to a zero vector.
    pub fn get_function_value(&self) -> &[f64] {
        &self.function_value_at_u
    }

    ///
    /// Evaluates a local Lipschitz constant of a given function
    ///
    /// Functions are closures of type `F` as shown here.
    ///
    /// Output arguments:
    ///
    /// - estimate of local Lipschitz constant (at point `u`)
    ///
    ///
    /// # Example
    ///
    /// ```
    /// let mut u = [1.0, 2.0, 3.0];
    /// let mut function_value = [0.0; 3];
    /// let f = |u: &[f64], g: &mut [f64]| -> i32 {
    ///    g[0] = 3.0 * u[0];
    ///    g[1] = 2.0 * u[1];
    ///    g[2] = 4.5;
    ///    0 };
    /// let mut lip_estimator =
    ///     panoc_rs::lipschitz_estimator::LipschitzEstimator::new(&mut u, &f, &mut function_value);
    /// let lip = lip_estimator.estimate_local_lipschitz();
    /// ```
    ///
    ///
    /// # Panics
    /// No rust-side panics, unless the C function which is called via this interface
    /// fails.
    ///
    pub fn estimate_local_lipschitz(&mut self) -> f64 {
        // function_value = gradient(u, p)
        (self.function)(self.u_decision_var, &mut self.function_value_at_u);
        let epsilon_lip = self.epsilon_lip;
        let delta_lip = self.delta_lip;

        // workspace = h = max{epsilon * u, delta}
        self.workspace
            .iter_mut()
            .zip(self.u_decision_var.iter())
            .for_each(|(out, &s)| {
                *out = if epsilon_lip * s > delta_lip {
                    epsilon_lip * s
                } else {
                    delta_lip
                }
            });
        let norm_h = norm2(&self.workspace);

        // u += workspace
        // u = u + h
        self.u_decision_var
            .iter_mut()
            .zip(self.workspace.iter())
            .for_each(|(out, a)| *out += *a);

        // workspace = F(u + h)
        (self.function)(self.u_decision_var, &mut self.workspace);

        // workspace = F(u + h) - F(u, p)
        self.workspace
            .iter_mut()
            .zip(self.function_value_at_u.iter())
            .for_each(|(out, a)| *out -= *a);

        let norm_workspace = norm2(&self.workspace);
        norm_workspace / norm_h
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    fn my_function(u: &[f64], g: &mut [f64]) -> i32 {
        g[0] = 3.0 * u[0];
        g[1] = 2.0 * u[1];
        g[2] = 4.5;
        0
    }

    #[test]
    fn test_lip_delta_epsilon_0() {
        let mut u: [f64; 3] = [1.0, 2.0, 3.0];
        let mut function_value = [0.0; 3];
        let f = |u: &[f64], g: &mut [f64]| -> i32 { my_function(u, g) };
        let mut lip_estimator = LipschitzEstimator::new(&mut u, &f, &mut function_value);
        lip_estimator.with_delta(1e-4).with_epsilon(1e-4);
        let lip = lip_estimator.estimate_local_lipschitz();
        assert!((1.336306209562331 - lip).abs() < 1e-12);
        println!("Lx = {}", lip);
    }

    #[test]
    #[should_panic]
    fn test_lip_delta_epsilon_panic1() {
        let mut u: [f64; 3] = [1.0, 2.0, 3.0];
        let mut function_value = [0.0; 3];
        let mut lip_estimator = LipschitzEstimator::new(&mut u, &my_function, &mut function_value);
        lip_estimator.with_epsilon(0.0);
    }

    #[test]
    #[should_panic]
    fn test_lip_delta_epsilon_panic2() {
        let mut u: [f64; 3] = [1.0, 2.0, 3.0];
        let mut function_value = [0.0; 3];
        let mut lip_estimator = LipschitzEstimator::new(&mut u, &my_function, &mut function_value);
        lip_estimator.with_epsilon(0.0);
    }

    #[test]
    fn test_lip_estimator_mock() {
        let mut u: [f64; 3] = [1.0, 2.0, 3.0];
        let f = |u: &[f64], g: &mut [f64]| -> i32 { my_function(u, g) };
        let mut function_value = [0.0; 3];
        let mut lip_estimator = LipschitzEstimator::new(&mut u, &f, &mut function_value);
        let lip = lip_estimator.estimate_local_lipschitz();
        assert!((1.3363062094165823 - lip).abs() < 1e-4);
        println!("L_mock = {}", lip);
    }

    #[test]
    fn test_norm() {
        // Rust allows to test private functions! Awesome!
        let p = [1.0, 2.0, 3.0];
        let norm_p = norm2(&p);
        println!("norm(p) = {}", norm_p);
        assert!((3.741657386773941 - norm_p).abs() < 1e-12);
    }

    #[test]
    fn test_get_function_value() {
        let u: [f64; 10] = [1.0, 2.0, 3.0, -5.0, 1.0, 10.0, 14.0, 17.0, 3.0, 5.0];
        let mut function_value = [0.0; 10];
        let mut u_copy = u.clone();
        let f = |u: &[f64], g: &mut [f64]| -> i32 { my_function(u, g) };
        let mut lip_estimator = LipschitzEstimator::new(&mut u_copy, &f, &mut function_value);
        {
            let computed_gradient = lip_estimator.get_function_value();
            computed_gradient
                .iter()
                .for_each(|&s| assert_eq!(s, 0.0_f64));
        }

        lip_estimator.estimate_local_lipschitz();

        let computed_gradient = lip_estimator.get_function_value();
        let mut actual_gradient = [0.0; 10];
        f(&u, &mut actual_gradient);

        computed_gradient
            .iter()
            .zip(actual_gradient.iter())
            .for_each(|(a, b)| assert_eq!(a, b));
    }
}
