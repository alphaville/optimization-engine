///
/// Auto-generated TCP server for optimizer: {{ meta.optimizer_name }}
///
use optimization_engine::alm::*;
use serde::{Deserialize, Serialize};

#[macro_use]
extern crate clap;

use std::{
    io::{prelude::Read, Write},
    net::TcpListener,
};

use clap::{Arg, App};

use {{ meta.optimizer_name }}::*;

#[macro_use]
extern crate log;

/// IP of server (use 0.0.0.0 to bind to any IP)
/// Can be overriden by the user
const BIND_IP_DEFAULT: &str = "{{tcp_server_config.bind_ip}}";

/// Port
/// Can be overriden by the user
const BIND_PORT_DEFAULT: u32 = {{tcp_server_config.bind_port}};

/// Size of read buffer
/// Can be overriden by the user
const READ_BUFFER_SIZE: usize = 1024;

/// Configuration of TCP server (provided by the user
/// as command-line parameters)
#[derive(Debug)]
struct TcpServerConfiguration<'a> {
   /// Bind IP
   ip: &'a str,
   /// Port
   port: u32,
}

#[derive(Deserialize, Debug)]
struct ExecutionParameter {
    /// Parameter
    parameter: Vec<f64>,
    /// Initial guess (can be null)
    initial_guess: Option<Vec<f64>>,
    /// Initial Lagrange multipliers (can be null)
    initial_lagrange_multipliers: Option<Vec<f64>>,
    /// Initial penalty parameter, c0
    initial_penalty: Option<f64>,
}

/// Request from the client
#[derive(Deserialize, Debug)]
enum ClientRequest {
    /// Command: run solver
    Run(ExecutionParameter),
    /// Command: ping (check if server is up)
    Ping(i32),
    /// Command: kill gracefully
    Kill(i32),
}

/// Solution and solution status of optimizer
#[derive(Serialize, Debug)]
struct OptimizerSolution<'a> {
    exit_status: String,
    num_outer_iterations: usize,
    num_inner_iterations: usize,
    last_problem_norm_fpr: f64,
    delta_y_norm_over_c: f64,
    f2_norm: f64,
    solve_time_ms: f64,
    penalty: f64,
    solution: &'a [f64],
    lagrange_multipliers: &'a [f64],
    cost: f64,
}

fn pong(stream: &mut std::net::TcpStream, code: i32) {
    let error_message = format!(
        {% raw %}"{{\n\t\"Pong\" : {}\n}}\n"{% endraw %},
        code
    );
    stream
        .write_all(error_message.as_bytes())
        .expect("cannot write to stream");
}

/// Writes an error to the communication stream
fn write_error_message(stream: &mut std::net::TcpStream, code: i32, error_msg: &str) {
    let error_message = format!(
        {% raw %}"{{\n\t\"type\" : \"Error\", \n\t\"code\" : {}, \n\t\"message\" : \"{}\"\n}}\n"{% endraw %},
        code,
        error_msg
    );
    warn!("Invalid request {:?}", code);
    stream
        .write_all(error_message.as_bytes())
        .expect("cannot write to stream");
}

/// Serializes the solution and solution status and returns it
/// to the client
fn return_solution_to_client(
    status: AlmOptimizerStatus,
    solution: &[f64],
    stream: &mut std::net::TcpStream,
) {
    let empty_vec : [f64; 0] = Default::default();
    let solution: OptimizerSolution = OptimizerSolution {
        exit_status: format!("{:?}", status.exit_status()),
        num_outer_iterations: status.num_outer_iterations(),
        num_inner_iterations: status.num_inner_iterations(),
        last_problem_norm_fpr: status.last_problem_norm_fpr(),
        delta_y_norm_over_c: status.delta_y_norm_over_c(),
        f2_norm: status.f2_norm(),
        penalty: status.penalty(),
        lagrange_multipliers: if let Some(y) = &status.lagrange_multipliers() { y } else { &empty_vec },
        solve_time_ms: (status.solve_time().as_nanos() as f64) / 1e6,
        solution,
        cost: status.cost(),

    };
    let solution_json = serde_json::to_string_pretty(&solution).unwrap();
    stream
        .write_all(solution_json.as_bytes())
        .expect("cannot write to stream");
}

/// Handles an execution request
fn execution_handler(
    cache: &mut AlmCache,
    execution_parameter: &ExecutionParameter,
    u: &mut [f64],
    p: &mut [f64],
    stream: &mut std::net::TcpStream,
) {
    // ----------------------------------------------------
    // Set initial value
    // ----------------------------------------------------
    let initial_guess = &execution_parameter.initial_guess;
    match initial_guess {
        None => {
            info!("no initial guess provided");
        },
        Some(u0) => {
            if u0.len() != {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES {
                warn!("initial guess has incompatible dimensions");
                write_error_message(stream, 1600, "Initial guess has incompatible dimensions");
                return;
            }
            u.copy_from_slice(u0);
        }
    }

    // ----------------------------------------------------
    // Check lagrange multipliers
    // ----------------------------------------------------
    if let Some(y0) = &execution_parameter.initial_lagrange_multipliers {
        if y0.len() != {{meta.optimizer_name|upper}}_N1 {
            write_error_message(stream, 1700, "wrong dimension of Langrange multipliers");
            return;
        }
    }

    // ----------------------------------------------------
    // Run solver
    // ----------------------------------------------------
    let parameter = &execution_parameter.parameter;
    if parameter.len() != {{meta.optimizer_name|upper}}_NUM_PARAMETERS {
        write_error_message(stream, 3003, "wrong number of parameters");
        return;
    }
    p.copy_from_slice(parameter);
    let status = solve(p,
                       cache,
                       u,
                       &execution_parameter.initial_lagrange_multipliers,
                       &execution_parameter.initial_penalty);
    match status {
        Ok(ok_status) => {
            return_solution_to_client(ok_status, u, stream);
        }
        Err(_) => {
            write_error_message(stream, 2000, "Problem solution failed (solver error)");
        }
    }
}

fn run_server(tcp_config: &TcpServerConfiguration) {
    info!("Initializing cache...");
    let mut p = [0.0; {{meta.optimizer_name|upper}}_NUM_PARAMETERS];
    let mut cache = initialize_solver();
    info!("Done");
    let listener = TcpListener::bind(format!("{}:{}", tcp_config.ip, tcp_config.port)).unwrap();
    let mut u = [0.0; {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES];
    info!("listening started, ready to accept connections at {}:{}", tcp_config.ip, tcp_config.port);
    for stream in listener.incoming() {
        let mut stream = stream.unwrap();

        //The following is more robust compared to `read_to_string`
        let mut bytes_buffer = vec![0u8; READ_BUFFER_SIZE];
        let mut read_data_length = 1;
        let mut buffer = String::new();
        while read_data_length != 0 {
            read_data_length = stream
                .read(&mut bytes_buffer)
                .expect("could not read stream");
            let new_string = String::from_utf8(bytes_buffer[0..read_data_length].to_vec())
                .expect("sent data is not UFT-8");
            buffer.push_str(&new_string);
        }

        let received_request: serde_json::Result<ClientRequest> = serde_json::from_str(&buffer);
        trace!("Received new request");
        match received_request {
            Ok(request_content) => match request_content {
                ClientRequest::Run(execution_param) => {
                    trace!("Running solver");
                    execution_handler(&mut cache,
                                      &execution_param,
                                      &mut u,
                                      &mut p,
                                      &mut stream);
                }
                ClientRequest::Kill(_) => {
                    info!("Quitting on request");
                    break;
                }
                ClientRequest::Ping(ping_code) => {
                    info!("Ping received");
                    pong(&mut stream, ping_code);
                }
            },
            Err(_) => {
                write_error_message(&mut stream, 1000, "Invalid request");
            }
        }
    }
}

fn main() {
    let matches = App::new("OpEn TCP Server [{{meta.optimizer_name}}]")
        .version("{{meta.version}}")
        .author("{{meta.authors | join(', ')}}")
        .about("TCP interface of OpEn optimizer")
        .arg(Arg::with_name("ip")
                 .short("ip")
                 .long("ip")
                 .takes_value(true)
                 .help("TCP server bind IP (0.0.0.0 for no restrictions)"))
        .arg(Arg::with_name("port")
                 .short("p")
                 .long("port")
                 .takes_value(true)
                 .help("TCP server port"))
        .get_matches();
    let port = value_t!(matches, "port", u32).unwrap_or(BIND_PORT_DEFAULT);
    let ip = matches.value_of("ip").unwrap_or(BIND_IP_DEFAULT);
    let server_config = TcpServerConfiguration {ip, port};

    pretty_env_logger::init();
    info!("{:?}", server_config);
    run_server(&server_config);
    info!("Exiting... (adios!)");
}
