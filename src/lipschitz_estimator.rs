#![deny(missing_docs)]
//!
//! Estimates a local Lipschitz constant for a mapping
//! $F: \mathbb{R}^n \to \mathbb{R}^n$
//!
//! Functions are provided as closures.
//!
//! # Method
//!
//! This method computes a numerical approximation of the norm of the directional
//! derivative of a function $F:\mathbb{R}^n \to \mathbb{R}^n$ at a point $u\in\mathbb{R}^n$
//! along a direction $h \in \mathbb{R}^n$ with $h_i = \max \\{\delta, \epsilon u_i\\}$,
//! where $\delta$ and $\epsilon$ are given small numbers.
//!
//! The estimated (local) Lipschitz constant is
//!
//! $$
//! L_F(u) = \frac{\Vert{}F(u + h) - F(u){}\Vert}{\Vert{}h{}\Vert}
//! $$
//!
//! # Example
//!
//! ```
//! use optimization_engine::{SolverError, lipschitz_estimator::LipschitzEstimator};
//!
//! pub fn F(u: &[f64], g: &mut [f64]) -> Result<(), SolverError> {
//!     g[0] = 3.0 * u[0];
//!     g[1] = 2.0 * u[1];
//!     g[2] = 4.5;
//!     Ok(())
//! }
//!
//! let mut u: [f64; 3] = [1.0, 2.0, 3.0];
//! let mut function_value = [0.0; 3];
//! let mut lip_estimator = LipschitzEstimator::new(&mut u, &F, &mut function_value);
//! let lip = lip_estimator.estimate_local_lipschitz();
//! ```
//!

use crate::{matrix_operations, SolverError};

const DEFAULT_DELTA: f64 = 1e-6;
const DEFAULT_EPSILON: f64 = 1e-6;

/// Structure for the computation of estimates of the Lipschitz constant of mappings
pub struct LipschitzEstimator<'a, F>
where
    F: Fn(&[f64], &mut [f64]) -> Result<(), SolverError>,
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

impl<'a, F> LipschitzEstimator<'a, F>
where
    F: Fn(&[f64], &mut [f64]) -> Result<(), SolverError>,
{
    /// Creates a new instance of this structure
    ///
    /// # Arguments
    ///
    /// - `u_` On entry: point where the Lipschitz constant is estimated,
    ///    On exit: the provided slice is modified (this is why it is a mutable
    ///    reference). The value of `u_` at exit is slightly perturbed. If you need
    ///    to keep the original value of `u_`, you need to make a copy of the variable
    ///    before you provide it to this method.
    /// - `f_` given closure
    /// - `function_value_` externally allocated memory which on exit stores the
    ///    value of the given function at `u_`, that is `f_(u_)`
    ///
    /// # Returns
    ///
    /// New instance of `LipschitzEstimator`
    ///
    ///
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
            epsilon_lip: DEFAULT_EPSILON,
            delta_lip: DEFAULT_DELTA,
        }
    }

    ///
    /// A setter method for `delta`
    ///
    /// # Arguments
    ///
    /// - `delta`: parameter delta (the default value is `1e-6`)
    ///
    /// # Panics
    /// The method will panic if `delta` is non positive
    ///
    pub fn with_delta(mut self, delta: f64) -> Self {
        assert!(delta > 0.0);
        self.delta_lip = delta;
        self
    }

    ///
    /// A setter method for `epsilon`
    ///
    /// # Arguments
    ///
    /// - `epsilon`: parameter epsilon  (the default value is `1e-6`)
    ///
    /// # Panics
    /// The method will panic if `epsilon` is non positive
    ///
    pub fn with_epsilon(mut self, epsilon: f64) -> Self {
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
    /// # Returns
    ///
    /// - estimate of local Lipschitz constant (at point `u`)
    ///
    ///
    /// # Example
    ///
    /// ```
    /// use optimization_engine::SolverError;
    ///
    /// let mut u = [1.0, 2.0, 3.0];
    /// let mut function_value = [0.0; 3];
    /// let f = |u: &[f64], g: &mut [f64]| -> Result<(), SolverError> {
    ///     g[0] = 3.0 * u[0];
    ///     g[1] = 2.0 * u[1];
    ///     g[2] = 4.5;
    ///     Ok(())
    /// };
    /// let mut lip_estimator =
    ///     optimization_engine::lipschitz_estimator::LipschitzEstimator::new(&mut u, &f, &mut function_value);
    /// let lip = lip_estimator.estimate_local_lipschitz();
    /// ```
    ///
    ///
    /// # Panics
    /// No rust-side panics, unless the C function which is called via this interface
    /// fails.
    ///
    pub fn estimate_local_lipschitz(&mut self) -> Result<f64, SolverError> {
        // function_value = gradient(u, p)
        (self.function)(self.u_decision_var, &mut self.function_value_at_u)?;
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
        let norm_h = matrix_operations::norm2(&self.workspace);

        // u += workspace
        // u = u + h
        self.u_decision_var
            .iter_mut()
            .zip(self.workspace.iter())
            .for_each(|(out, a)| *out += *a);

        // workspace = F(u + h)
        (self.function)(self.u_decision_var, &mut self.workspace)?;

        // workspace = F(u + h) - F(u)
        self.workspace
            .iter_mut()
            .zip(self.function_value_at_u.iter())
            .for_each(|(out, a)| *out -= *a);

        let norm_workspace = matrix_operations::norm2(&self.workspace);
        Ok(norm_workspace / norm_h)
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::mocks;

    #[test]
    fn t_test_lip_delta_epsilon_0() {
        let mut u: [f64; 3] = [1.0, 2.0, 3.0];
        let mut function_value = [0.0; 3];

        let f =
            |u: &[f64], g: &mut [f64]| -> Result<(), SolverError> { mocks::lipschitz_mock(u, g) };

        let mut lip_estimator = LipschitzEstimator::new(&mut u, &f, &mut function_value)
            .with_delta(1e-4)
            .with_epsilon(1e-4);
        let lip = lip_estimator.estimate_local_lipschitz().unwrap();

        unit_test_utils::assert_nearly_equal(
            1.336_306_209_562_331,
            lip,
            1e-10,
            1e-14,
            "lipschitz constant",
        );

        println!("Lx = {}", lip);
    }

    #[test]
    #[should_panic]
    fn t_test_lip_delta_epsilon_panic1() {
        let mut u: [f64; 3] = [1.0, 2.0, 3.0];
        let mut function_value = [0.0; 3];

        let _lip_estimator =
            LipschitzEstimator::new(&mut u, &mocks::lipschitz_mock, &mut function_value)
                .with_epsilon(0.0);
    }

    #[test]
    #[should_panic]
    fn t_test_lip_delta_epsilon_panic2() {
        let mut u: [f64; 3] = [1.0, 2.0, 3.0];
        let mut function_value = [0.0; 3];

        let _lip_estimator =
            LipschitzEstimator::new(&mut u, &mocks::lipschitz_mock, &mut function_value)
                .with_epsilon(0.0);
    }

    #[test]
    fn t_test_lip_estimator_mock() {
        let mut u: [f64; 3] = [1.0, 2.0, 3.0];

        let f =
            |u: &[f64], g: &mut [f64]| -> Result<(), SolverError> { mocks::lipschitz_mock(u, g) };

        let mut function_value = [0.0; 3];

        let mut lip_estimator = LipschitzEstimator::new(&mut u, &f, &mut function_value);
        let lip = lip_estimator.estimate_local_lipschitz().unwrap();

        unit_test_utils::assert_nearly_equal(
            1.336_306_209_416_582_3,
            lip,
            1e-8,
            1e-14,
            "lipschitz constant",
        );
        println!("L_mock = {}", lip);
    }

    #[test]
    fn t_test_get_function_value() {
        let u: [f64; 10] = [1.0, 2.0, 3.0, -5.0, 1.0, 10.0, 14.0, 17.0, 3.0, 5.0];
        let mut function_value = [0.0; 10];
        let mut u_copy = u;

        let f =
            |u: &[f64], g: &mut [f64]| -> Result<(), SolverError> { mocks::lipschitz_mock(u, g) };

        let mut lip_estimator = LipschitzEstimator::new(&mut u_copy, &f, &mut function_value);
        {
            let computed_gradient = lip_estimator.get_function_value();

            unit_test_utils::assert_nearly_equal_array(
                &[0.0; 10],
                &computed_gradient,
                1e-10,
                1e-14,
                "computed gradient",
            );
            computed_gradient.iter().for_each(|&s| {
                unit_test_utils::assert_nearly_equal(0.0_f64, s, 1e-10, 1e-16, "gradient")
            });
        }

        lip_estimator.estimate_local_lipschitz().unwrap();

        let computed_gradient = lip_estimator.get_function_value();
        let mut actual_gradient = [0.0; 10];
        f(&u, &mut actual_gradient).unwrap();

        unit_test_utils::assert_nearly_equal_array(
            &computed_gradient,
            &actual_gradient,
            1e-10,
            1e-14,
            "computed/actual gradient",
        );
    }
}
