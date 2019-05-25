# Rust Casadi Interface
[![Build Status](https://travis-ci.org/alphaville/icasadi.svg?branch=master)](https://travis-ci.org/alphaville/icasadi)

This is an interface to CasADi functions of the form `phi(u; p)`, where `u` is a decision variable and `p` a parameter.

- Using CasADi's MATLAB or Python interface, you may define a cost function
- We provide helper functions which generate C code for the given function and its Jacobian
- Then `icasadi` offers a convenient interface to the C code from Rust
- This is a `no-std` library which can be used on embedded devices
- And `icasadi` can be used in embedded numerical optimization modules written in Rust

This library is available on [crates.io](https://crates.io/crates/icasadi) at https://crates.io/crates/icasadi

## Code generation in Python

Coming very soon


## Code generation in MATLAB
Here is an example of such a function (MATLAB example)

```matlab
% File: matlab/example.m
nu = 10;                           % number of decision variables
np = 2;                            % number of parameters 

u = casadi.SX.sym('u', nu);        % decision variables
p = casadi.SX.sym('p', np);        % parameters

phi = (p'*p) * cos(sin(u))' * u;   % cost function phi(u; p)
```

We may then create C code for this function and its Jacobian using

```matlab
[cost, grad_cost] = casadi_generate_c_code(u, p, phi);
```


This will create two functions:

- `cost` : which maps `(u, p)` to `phi(u; p)`,
- `grad_cost` : the Jacobian matrix of `phi` with respect to `u` evaluated 
   at `(u, p)`


Here is an example of use:

```rust
// File: main.rs
extern crate icasadi;

fn main() {
    let u = [1.0, 2.0, 3.0, -5.0, 1.0, 10.0, 14.0, 17.0, 3.0, 5.0];
    let p = [1.0, -1.0];

    let mut cost_value = 0.0;
    let mut jac = [0.0; 10];
    
    icasadi_cost(u, p, &phival);       // compute the cost
    icasadi_grad(u, p, cost_jacobian); // compute the Jacobian of the cost

    println!("cost value = {}", cost_value);
    println!("jacobian   = {:#?}", jac);
}
```

## Compiling, Running, Testing

To build the project, run

```
$ cargo build
```

To compile the main function (`main.rs`), run

```
$ cargo run
```

To run the unit tests, do

```
$ cargo test
```
