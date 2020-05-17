use crate::{matrix_operations, SolverError};

pub const SOLUTION_A: [f64; 2] = [-0.148_959_718_255_77, 0.133_457_867_273_39];
pub const SOLUTION_HARD: [f64; 3] = [
    -0.041_123_164_672_281,
    -0.028_440_417_469_206,
    0.000_167_276_757_790,
];

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

pub fn rosenbrock_cost(a: f64, b: f64, u: &[f64]) -> f64 {
    (a - u[0]).powi(2) + b * (u[1] - u[0].powi(2)).powi(2)
}

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
pub fn psi_cost_dummy(u: &[f64], xi: &[f64], cost: &mut f64) -> Result<(), SolverError> {
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

/// Gradient of `psi_cost`
pub fn psi_gradient_dummy(u: &[f64], xi: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
    let u_len = u.len();
    let xi_len = xi.len();
    assert!(
        u_len > xi_len,
        "the length of u must be larger than that of xi"
    );
    assert!(u_len == grad.len(), "u and grad must have equal lengths");
    grad.copy_from_slice(u);
    grad.iter_mut().for_each(|grad_i| *grad_i += xi[0]);
    xi[1..]
        .iter()
        .zip(grad.iter_mut())
        .for_each(|(xi_i, grad_i)| *grad_i += xi_i);
    Ok(())
}

/// A mock affine mapping given by
///
/// ```
///  F1(u1, u2, u3) = [2*u1 + u3 - 1
///                      u1 + 3*u2  ]
/// ```
///
/// It is `F1: R^3 --> R^2`
///
pub fn mapping_f1_affine(u: &[f64], f1u: &mut [f64]) -> Result<(), SolverError> {
    assert!(u.len() == 3, "the length of u must be equal to 3");
    assert!(f1u.len() == 2, "the length of F1(u) must be equal to 2");
    f1u[0] = 2.0 * u[0] + u[2] - 1.0;
    f1u[1] = u[0] + 3.0 * u[1];
    Ok(())
}

/// Compute the product `JF1(u)'*d`, for a given 2-vector `d`, where
/// `F1` is the affine mapping given by `mapping_f1_affine`
///
/// This is
///
/// ```
/// JF1(x)'*d = [2*d1 + d2
///              3*d2
///              d1        ]
/// ```
///  
pub fn mapping_f1_affine_jacobian_product(
    _u: &[f64],
    d: &[f64],
    res: &mut [f64],
) -> Result<(), SolverError> {
    assert!(d.len() == 2, "the length of d must be equal to 3");
    assert!(res.len() == 3, "the length of res must be equal to 3");
    res[0] = 2.0 * d[0] + d[1];
    res[1] = 3.0 * d[1];
    res[2] = d[0];
    Ok(())
}

/// Simple quadratic cost function
///
/// ```
/// f0(u) = 0.5*u'*u + 1'*u
/// ```
pub fn f0(u: &[f64], cost: &mut f64) -> Result<(), SolverError> {
    *cost = 0.5 * matrix_operations::norm2_squared(u) + matrix_operations::sum(u);
    Ok(())
}

pub fn d_f0(u: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
    grad.iter_mut()
        .zip(u.iter())
        .for_each(|(grad_i, u_i)| *grad_i = u_i + 1.0);
    Ok(())
}

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
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
        assert!(psi_cost_dummy(&u, &xi, &mut cost).is_ok());
        unit_test_utils::assert_nearly_equal(83.5, cost, 1e-16, 1e-12, "psi_cost is wrong");
    }

    #[test]
    fn t_psi_gradient() {
        let u = [-1.0, 2.0, -3.0, 4.0, 5.0];
        let xi = [15., 20., 11., 17.];
        let mut grad = vec![0.0; 5];
        assert!(psi_gradient_dummy(&u, &xi, &mut grad).is_ok());
        println!("grad = {:?}", grad);
        unit_test_utils::assert_nearly_equal_array(
            &grad,
            &[34.0, 28.0, 29.0, 19.0, 20.0],
            1e-12,
            1e-12,
            "psi_grad is wrong",
        );
    }

    #[test]
    fn t_mapping_f1_affine() {
        let u = [5.0, 2.0, 3.0];
        let mut f1u = [0.0; 2];
        assert!(mapping_f1_affine(&u, &mut f1u).is_ok());
        unit_test_utils::assert_nearly_equal_array(&f1u, &[12., 11.], 1e-12, 1e-12, "f1 is wrong");
    }

    #[test]
    fn t_mapping_f1_affine_jacobian_product() {
        let d = [5.0, -10.0];
        let mut jac_f1_trans_times_d = [0.0; 3];
        assert!(mapping_f1_affine_jacobian_product(&[], &d, &mut jac_f1_trans_times_d).is_ok());
        println!("jac = {:?}", &jac_f1_trans_times_d);
        unit_test_utils::assert_nearly_equal_array(
            &jac_f1_trans_times_d,
            &[0., -30., 5.],
            1e-10,
            1e-10,
            "jacobian result is wrong",
        );
    }

    #[test]
    fn t_f0() {
        let u = [3.0, 5.0, 7.0];
        let mut cost = 0.0;
        assert!(f0(&u, &mut cost).is_ok());
        unit_test_utils::assert_nearly_equal(56.5, cost, 1e-16, 1e-12, "f0(u) is wrong");
    }

    #[test]
    fn t_d_f0() {
        let u = [3.0, -5.0, 7.0];
        let mut grad = [0.0; 3];
        assert!(d_f0(&u, &mut grad).is_ok());
        unit_test_utils::assert_nearly_equal_array(
            &[4., -4., 8.],
            &grad,
            1e-16,
            1e-12,
            "d_f0 is wrong",
        );
    }
}
