use crate::{matrix_operations, SolverError};

pub const SOLUTION_A: [f64; 2] = [-0.14895971825577, 0.13345786727339];
pub const SOLUTION_HARD: [f64; 3] = [-0.041123164672281, -0.028440417469206, 0.000167276757790];

pub fn lipschitz_mock(u: &[f64], g: &mut [f64]) -> Result<(), SolverError> {
    g[0] = 3.0 * u[0];
    g[1] = 2.0 * u[1];
    g[2] = 4.5;
    Ok(())
}

pub fn void_parameteric_cost(_u: &[f64], _p: &[f64], _cost: &mut f64) -> Result<(), SolverError> {
    Ok(())
}

pub fn void_parameteric_gradient(
    _u: &[f64],
    _p: &[f64],
    _grad: &mut [f64],
) -> Result<(), SolverError> {
    Ok(())
}

pub fn void_mapping(_u: &[f64], _result: &mut [f64]) -> Result<(), SolverError> {
    Ok(())
}

pub fn void_cost(_u: &[f64], _cost: &mut f64) -> Result<(), SolverError> {
    Ok(())
}

pub fn void_gradient(_u: &[f64], _grad: &mut [f64]) -> Result<(), SolverError> {
    Ok(())
}

pub fn my_cost(u: &[f64], cost: &mut f64) -> Result<(), SolverError> {
    *cost = 0.5 * (u[0].powi(2) + 2. * u[1].powi(2) + 2.0 * u[0] * u[1]) + u[0] - u[1] + 3.0;
    Ok(())
}

pub fn my_gradient(u: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
    grad[0] = u[0] + u[1] + 1.0;
    grad[1] = u[0] + 2. * u[1] - 1.0;
    Ok(())
}

#[allow(dead_code)]
pub fn rosenbrock_cost(a: f64, b: f64, u: &[f64]) -> f64 {
    (a - u[0]).powi(2) + b * (u[1] - u[0].powi(2)).powi(2)
}

#[allow(dead_code)]
pub fn rosenbrock_grad(a: f64, b: f64, u: &[f64], grad: &mut [f64]) {
    grad[0] = 2.0 * u[0] - 2.0 * a - 4.0 * b * u[0] * (-u[0].powi(2) + u[1]);
    grad[1] = b * (-2.0 * u[0].powi(2) + 2.0 * u[1]);
}

pub fn hard_quadratic_cost(u: &[f64], cost: &mut f64) -> Result<(), SolverError> {
    *cost = (4. * u[0].powi(2)) / 2.
        + 5.5 * u[1].powi(2)
        + 500.5 * u[2].powi(2)
        + 5. * u[0] * u[1]
        + 25. * u[0] * u[2]
        + 5. * u[1] * u[2]
        + u[0]
        + u[1]
        + u[2];
    Ok(())
}

pub fn hard_quadratic_gradient(u: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
    // norm(Hessian) = 1000.653 (Lipschitz gradient)
    grad[0] = 4. * u[0] + 5. * u[1] + 25. * u[2] + 1.;
    grad[1] = 5. * u[0] + 11. * u[1] + 5. * u[2] + 1.;
    grad[2] = 25. * u[0] + 5. * u[1] + 1001. * u[2] + 1.;
    Ok(())
}

/// Parameteric cost function `psi(u; xi)` given by
///
/// `phi(u, xi) = 0.5*u'*u + xi[0]*sum(u) + xi[1..m]'*u[0..m-1]`
///
/// where `m` is the length of `xi`. It is assumed that the length of
/// `u` is larger than the length of `xi`
pub fn psi_cost(u: &[f64], xi: &[f64], cost: &mut f64) -> Result<(), SolverError> {
    let u_len = u.len();
    let xi_len = xi.len();
    assert!(u_len > xi_len);
    let sum_u = u.iter().fold(0.0, |mut sum, ui| {
        sum += ui;
        sum
    });
    // psi_cost = 0.5*SUM(ui^2) + xi[0] * sum_u
    *cost =
        0.5 * u.iter().fold(0.0, |mut sum_of_squares, ui| {
            sum_of_squares += ui.powi(2);
            sum_of_squares
        }) + xi[0] * sum_u;
    // psi_cost += xi[1..m]'*u[0..m-1]
    let m = std::cmp::min(u_len, xi_len - 1);
    *cost += matrix_operations::inner_product(&u[..m], &xi[1..=m]);
    Ok(())
}

pub fn psi_gradient(u: &[f64], xi: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
    let u_len = u.len();
    let xi_len = xi.len();
    assert!(
        u_len > xi_len,
        "the length of u must be larger than that of xi"
    );
    assert!(u_len == grad.len(), "u and grad must have equal lengths");
    grad.copy_from_slice(u);
    grad.iter_mut().for_each(|grad_i| *grad_i += xi[0]);
    &xi[1..]
        .iter()
        .zip(grad.iter_mut())
        .for_each(|(xi_i, grad_i)| *grad_i += xi_i);
    Ok(())
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn t_mock_hard() {
        let x = [1.5, 2.6, -3.7];
        let mut df = [0.0; 3];
        let mut cost = 0.0;
        assert_eq!(Ok(()), hard_quadratic_cost(&x, &mut cost));

        unit_test_utils::assert_nearly_equal(6726.575, cost, 1e-8, 1e-10, "cost");

        assert_eq!(Ok(()), hard_quadratic_gradient(&x, &mut df));
        unit_test_utils::assert_nearly_equal_array(
            &[-72.5, 18.6, -3652.2],
            &df,
            1e-6,
            1e-6,
            "grad",
        );
    }

    #[test]
    fn t_psi_cost() {
        let u = [-1.0, 2.0, -3.0, 4.0, 5.0];
        let xi = [15., 20., 11., 17.];
        let mut cost = 0.0;
        assert!(psi_cost(&u, &xi, &mut cost).is_ok());
        unit_test_utils::assert_nearly_equal(83.5, cost, 1e-16, 1e-12, "psi_cost is wrong");
    }

    #[test]
    fn t_psi_gradient() {
        let u = [-1.0, 2.0, -3.0, 4.0, 5.0];
        let xi = [15., 20., 11., 17.];
        let mut grad = vec![0.0; 5];
        assert!(psi_gradient(&u, &xi, &mut grad).is_ok());
        println!("grad = {:?}", grad);
        unit_test_utils::assert_nearly_equal_array(
            &grad,
            &[34.0, 28.0, 29.0, 19.0, 20.0],
            1e-12,
            1e-12,
            "psi_grad is wrong",
        );
    }
}
