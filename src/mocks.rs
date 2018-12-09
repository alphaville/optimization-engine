pub const SOLUTION: [f64; 2] = [-0.14895971825577, 0.13345786727339];

pub fn my_cost(u: &[f64], cost: &mut f64) -> i32 {
        *cost = 0.5 * (u[0].powi(2) + 2. * u[1].powi(2) + 2.0 * u[0] * u[1]) + u[0] - u[1] + 3.0;
        0
}

pub fn my_gradient(u: &[f64], grad: &mut [f64]) -> i32 {
        grad[0] = u[0] + u[1] + 1.0;
        grad[1] = u[0] + 2. * u[1] - 1.0;
        0
}

pub fn lipschitz_mock(u: &[f64], g: &mut [f64]) -> i32 {
        g[0] = 3.0 * u[0];
        g[1] = 2.0 * u[1];
        g[2] = 4.5;
        0
}
