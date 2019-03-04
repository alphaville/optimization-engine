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

fn get_cache() -> PANOCCache {
    let nu = icasadi::num_decision_variables();
    println!("NU = {}", nu);
    let mut panoc_cache: PANOCCache = PANOCCache::new(
        NonZeroUsize::new(nu).unwrap(),
        1e-6,
        NonZeroUsize::new(10).unwrap(),
    );
    panoc_cache
}

// call with:
// printf "{"'"'"parameter"'"'":[1.0,2.0,5.0]}" > /dev/udp/localhost/3498
fn main() {
    let mut cache = get_cache();

    use std::net::UdpSocket;
    let socket = UdpSocket::bind("0.0.0.0:3498").expect("couldn't bind to address");
    let mut buf = [0; 100];
    loop {
        let (number_of_bytes, src_addr) = socket.recv_from(&mut buf).expect("didn't receive data");
        let filled_buf = &mut buf[..number_of_bytes];
        let data = std::str::from_utf8(filled_buf).unwrap();
        let received_request: Result<OptimizationRequest> = serde_json::from_str(data);
        println!("Message: {:?}", data);
        println!("Who's asking... {:?}", src_addr);
        println!("Decoded parameter = {:?}", received_request.is_ok());


        let p = received_request.unwrap().parameter;
        let mut u = [0.0; 2];

        let df = |u: &[f64], grad: &mut [f64]| -> i32 {
            icasadi::icasadi_grad(u, &p, grad);
            0
        };
        let f = |u: &[f64], c: &mut f64| -> i32 { icasadi::icasadi_cost(u, &p, c) };

        let bounds = Ball2::new_at_origin_with_radius(10.0);
        let problem = Problem::new(bounds, df, f);
        let mut panoc_engine = PANOCEngine::new(problem, &mut cache);

        let mut panoc = PANOCOptimizer::new(&mut panoc_engine);
        panoc.with_max_iter(1000);

        let status = panoc.solve(&mut u);

        println!(
            "iterations = {}\nfrp = {}\nu = {:?}\n\n",
            status.get_number_iterations(),
            status.get_norm_fpr(),
            u
        );
    }
}
