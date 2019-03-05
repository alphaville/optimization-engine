extern crate optimization_engine;

use optimization_engine::constraints::*;
use optimization_engine::core::panoc::*;
use optimization_engine::core::*;
use std::net::UdpSocket;
use std::num::NonZeroUsize;

use serde::{Deserialize, Serialize};
use serde_json::Result;

#[derive(Serialize, Deserialize, Debug)]
struct OptimizationRequest {
    parameter: Vec<f64>,
}

const TOLERANCE: f64 = 1e-5;
const LBFGS_MEMORY: usize = 5;
const MAX_ITERS: usize = 500;
const NU: usize = 4;

fn get_cache() -> PANOCCache {
    let nu = icasadi::num_decision_variables();
    let panoc_cache: PANOCCache = PANOCCache::new(
        NonZeroUsize::new(nu).unwrap(),
        TOLERANCE,
        NonZeroUsize::new(LBFGS_MEMORY).unwrap(),
    );
    panoc_cache
}

// call with:
// printf "{"'"'"parameter"'"'":[1.0,2.0,5.0]}" > /dev/udp/localhost/3498
fn main() {
    let mut cache = get_cache();
    let socket = UdpSocket::bind("0.0.0.0:3498").expect("couldn't bind to address");
    let mut buf = [0; 100];
    let mut u = [0.0; NU];
    loop {
        let (number_of_bytes, src_addr) = socket.recv_from(&mut buf).expect("didn't receive data");
        let filled_buf = &mut buf[..number_of_bytes];
        let data = std::str::from_utf8(filled_buf).unwrap();
        let received_request: Result<OptimizationRequest> = serde_json::from_str(data);

        if !received_request.is_ok() {
            continue;
        }
        let p = received_request.unwrap().parameter;

        let df = |u: &[f64], grad: &mut [f64]| -> i32 {
            icasadi::icasadi_grad(u, &p, grad);
            0
        };
        let f = |u: &[f64], c: &mut f64| -> i32 { icasadi::icasadi_cost(u, &p, c) };

        let bounds = Ball2::new_at_origin_with_radius(50.0);
        let problem = Problem::new(bounds, df, f);
        let mut panoc_engine = PANOCEngine::new(problem, &mut cache);
        let mut panoc = PANOCOptimizer::new(&mut panoc_engine);
        panoc.with_max_iter(MAX_ITERS);

        let now = std::time::Instant::now();
        let status = panoc.solve(&mut u);

        let msg = format!(
            "{{\n\t\"p\" : {:?},\n\t\"u\" : {:.10?},\n\t\"n\" : {},\n\t\"f\" : {},\n\t\"dt\" : \"{:?}\"\n}}\n\n\n",
            p,
            u,
            status.get_number_iterations(),
            status.get_norm_fpr().log10(),
            now.elapsed()
        );
        let _result = socket.send_to(msg.as_bytes(), &src_addr);
    }
}
