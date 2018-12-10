pub const SOLUTION: [f64; 2] = [-0.14895971825577, 0.13345786727339];

pub fn lipschitz_mock(u: &[f64], g: &mut [f64]) -> i32 {
        g[0] = 3.0 * u[0];
        g[1] = 2.0 * u[1];
        g[2] = 4.5;
        0
}

pub fn void_cost(_u: &[f64], _cost: &mut f64) -> i32 {
        0
}

pub fn void_gradient(_u: &[f64], _grad: &mut [f64]) -> i32 {
        0
}

pub fn my_cost(u: &[f64], cost: &mut f64) -> i32 {
        *cost = 0.5 * (u[0].powi(2) + 2. * u[1].powi(2) + 2.0 * u[0] * u[1]) + u[0] - u[1] + 3.0;
        0
}

pub fn my_gradient(u: &[f64], grad: &mut [f64]) -> i32 {
        grad[0] = u[0] + u[1] + 1.0;
        grad[1] = u[0] + 2. * u[1] - 1.0;
        0
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

pub fn hard_quadratic_cost(u: &[f64], cost: &mut f64) -> i32 {
        *cost = 0.5 * (-3.0749 * u[0] * u[0] + 0.6075 * u[1] * u[1] + 102.5774 * u[2] * u[2])
                - 5.1359 * u[0] * u[1]
                + 1.7766 * u[1] * u[2]
                - 15.2719 * u[0] * u[2]
                + u[0]
                + u[1]
                + u[2];
        0
}

pub fn hard_quadratic_gradient(u: &[f64], grad: &mut [f64]) -> i32 {
        grad[0] = -3.0749 * u[0] - 5.1359 * u[1] - 15.2719 * u[2] + 1.0;
        grad[1] = 0.3195 * u[0] + 0.6075 * u[1] + 1.7766 * u[2] + 1.0;
        grad[2] = 21.3205 * u[0] + 34.4927 * u[1] + 102.5774 * u[2] + 1.0;
        0
}
