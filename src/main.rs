extern crate optimization_engine;

use optimization_engine::constraints::*;
use optimization_engine::core::panoc::*;
use optimization_engine::core::*;
use std::net::UdpSocket;
use std::num::NonZeroUsize;

pub fn rosenbrock_cost(a: f64, b: f64, u: &[f64]) -> f64 {
    (a - u[0]).powi(2) + b * (u[1] - u[0].powi(2)).powi(2)
}

#[allow(dead_code)]
pub fn rosenbrock_grad(a: f64, b: f64, u: &[f64], grad: &mut [f64]) {
    grad[0] = 2.0 * u[0] - 2.0 * a - 4.0 * b * u[0] * (-u[0].powi(2) + u[1]);
    grad[1] = b * (-2.0 * u[0].powi(2) + 2.0 * u[1]);
}

fn main() -> std::io::Result<()> {
    let tolerance = 1e-6;
    let mut a = 1.0;
    let mut b = 100.0;
    let n = 2;
    let lbfgs_memory = 10;
    let max_iters = 100;
    let mut u = [-1.5, 0.9];
    let mut panoc_cache = PANOCCache::new(
        NonZeroUsize::new(n).unwrap(),
        tolerance,
        NonZeroUsize::new(lbfgs_memory).unwrap(),
    );
    let mut radius = 1.0;
    let mut i = 0;
    while i < 100 {
        b *= 1.01;
        a -= 1e-3;
        radius += 0.001;
        let df = |u: &[f64], grad: &mut [f64]| -> i32 {
            rosenbrock_grad(a, b, u, grad);
            0
        };
        let f = |u: &[f64], c: &mut f64| -> i32 {
            *c = rosenbrock_cost(a, b, u);
            0
        };
        let bounds = Ball2::new_at_origin_with_radius(radius);
        let problem = Problem::new(bounds, df, f);
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);
        let mut panoc = PANOCOptimizer::new(&mut panoc_engine);
        panoc.with_max_iter(max_iters);

        let status: SolverStatus = panoc.solve(&mut u);

        i += 1;

        println!(
            "parameters: (a={:.4}, b={:.4}, r={:.4}), iters = {}",
            a,
            b,
            radius,
            status.get_number_iterations()
        );
        println!("u = {:#.6?}", u);
    }
    loop {
        let socket = UdpSocket::bind("0.0.0.0:3498")?;

        println!("Welcome to OpEn server");
        // Receives a single datagram message on the socket. If `buf` is too small to hold
        // the message, it will be cut off.
        let mut buf = [0; 10];
        let (amt, src) = socket.recv_from(&mut buf)?;

        // Redeclare `buf` as slice of the received data and send reverse data back to origin.
        let buf = &mut buf[..amt];
        buf.reverse();
        socket.send_to(buf, &src)?;
        println!("{:?}", std::str::from_utf8(buf));
    } // the socket is closed here
}
