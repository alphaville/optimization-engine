---
id: openrust-alm
title: Advanced Rust
description: OpEn in Rust and the augmented Lagrangian method
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.1/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

Here we shall go through a complete example on how to use Optimization Engine to solve
more general problems that involve constraints of the general form $F_1(u) \in C$ and 
$F_2(u) = 0$.

## Problem Definition 
Let us start by defining the smooth cost function $f:\mathbb{R}^{3}\to\mathbb{R}$ 
and its gradient. Suppose that 
$$f(u) = \tfrac{1}{2}\Vert{}u\Vert^2 + u_1+u_2+u_3$$
The function and its gradient, 
$$
\nabla f(u) = u + 1_{3}
$$

```rust
// Smooth cost function
pub fn f(u: &[f64], cost: &mut f64) -> Result<(), SolverError> {
    *cost = 0.5 * matrix_operations::norm2_squared(u) 
              + matrix_operations::sum(u);
    Ok(())
}
```

```rust
/// Gradient of f
pub fn df(u: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
    grad.iter_mut()
        .zip(u.iter())
        .for_each(|(grad_i, u_i)| *grad_i = u_i + 1.0);
    Ok(())
}
```

Suppose we need to impose the constraint $F_1(u) \in C$, where $C=\\{z\in\mathbb{R}^2: \Vert{}z{}\Vert \leq 1\\}$
and $F_1$ is the mapping 
$$
F_1(u)=\begin{bmatrix}2u_1 + u_1 + 0.5 \\\\ u_1+3u_2\end{bmatrix}
$$

```rust
// Mapping f1
pub fn f1(u: &[f64], f1u: &mut [f64]) -> Result<(), SolverError> {
    f1u[0] = 2.0 * u[0] + u[2] + 0.5;
    f1u[1] = u[0] + 3.0 * u[1];
    Ok(())
}
```

Next, we need to define the mapping $(u, d) \mapsto JF_1(u)^\top d$, where $JF_1$ is the 
Jacobian matrix of $F_1$ for given vectors $u\in\mathbb{R}^3$ and $d\in\mathbb{R}^2$, which 
in this case is given by 
$$
JF_1(u)^\top d = \begin{bmatrix}
2d_1 + d_2
\\\\
3d_2
\\\\
d_1 \end{bmatrix}
$$
This is computed by the following function
```rust
pub fn f1_jacobian_product(
    _u: &[f64],
    d: &[f64],
    res: &mut [f64],
) -> Result<(), SolverError> {
    res[0] = 2.0 * d[0] + d[1];
    res[1] = 3.0 * d[1];
    res[2] = d[0];
    Ok(())
}
```

The problem we need to solve is

<div class="math">
\[\begin{split}\mathbb{P}(p) {}:{} \operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{n_u}}&amp;\ \tfrac{1}{2}\Vert{}u\Vert^2 + u_1+u_2+u_3\\
\mathrm{subject\ to}\ \  &amp;-\tfrac{1}{2} \leq u_i \leq \tfrac{1}{2}\\
&amp; F_1(u, p) \in C\end{split}\]</div>


## Invoking the ALM/PM solver

We have now defined all problem components and we can solve the problem using the
ALM/PM solver ([`AlmOptimizer`]). However, the solver requires that we provide the 
cost function of the inner problem, that is, function $\psi$. When using the code
generation interfaces (e.g., the [Python interface]) this function is constructed
automatically and the user does not need to specify it explicitly. In Rust we may use
[`AlmFactory`] which can construct $\psi$ and its gradient, which we shall then 
pass on to [`AlmOptimizer`] to solve our problem. 

Let us see how this works throuh the following example:

```rust
let tol = 1e-5;    // PANOC (inner) tolerance
let nx = 3;        // number of variables
let n1 = 2;        // dimension of F1 (ALM-type constraints)
let n2 = 0;        // dimension of F2 (no PM-type constraints)
let lbfgs_mem = 3; // length of L-BFGS memory
let panoc_cache = PANOCCache::new(nx, tol, lbfgs_mem);  // PANOC Cache
let mut alm_cache = AlmCache::new(panoc_cache, n1, n2); // ALM Cache

let set_c = Ball2::new(None, 1.0);    // Set C
let bounds = Ball2::new(None, 0.5);   // Set U
let set_y = Ball2::new(None, 1e12);   // Set Y (convex, compact)

// AlmFactory constructs function `psi`, which is needed by the optimizer
let factory = AlmFactory::new(
    f,
    df,
    Some(f1),
    Some(f1_jacobian_product),
    NO_MAPPING,
    NO_JACOBIAN_MAPPING,
    Some(set_c),
    n2,
);

let alm_problem = AlmProblem::new(
    bounds,
    Some(set_c),
    Some(set_y),
    |u: &[f64], xi: &[f64], cost: &mut f64| -> Result<(), SolverError> {
        factory.psi(u, xi, cost)
    },
    |u: &[f64], xi: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        factory.d_psi(u, xi, grad)
    },
    Some(f1),
    NO_MAPPING,
    n1,
    n2,
);
```

We may now construct an instance of [`AlmOptimizer`] to which we specify the 
various configuration and tuning parameters of the numerical method, and we 
call method `solve`:

```rust
let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
    .with_delta_tolerance(1e-5)
    .with_max_outer_iterations(20)
    .with_epsilon_tolerance(1e-6)
    .with_initial_inner_tolerance(1e-2)
    .with_inner_tolerance_update_factor(0.5)
    .with_initial_penalty(100.0)
    .with_penalty_update_factor(1.05)
    .with_sufficient_decrease_coefficient(0.2)
    .with_initial_lagrange_multipliers(&vec![5.0; n1]);

let mut u = vec![0.0; nx]; // initial guess for `u`
let solver_result = alm_optimizer.solve(&mut u);
let r = solver_result.unwrap();
println!("\n\n{:#.7?}\n", r);
```

## Result

The above program will print

```rust
AlmOptimizerStatus {
    exit_status: Converged,
    num_outer_iterations: 15,
    num_inner_iterations: 238,
    last_problem_norm_fpr: 0.0000014,
    lagrange_multipliers: Some(
        [
            -0.3074545,
            -0.3033940,
        ],
    ),
    solve_time: 2.4056250ms,
    penalty: 188.5649142,
    delta_y_norm: 0.0000038,
    f2_norm: 0.0000000,
}
```

We see that the solver performed 15 outer iterations, 238 inner iterations in total,
the algorithm converged within the specified tolerances (see `exit_status`) in 2.4 
milliseconds and the vector of Lagrange multipliers at the solution is 
$$
y^\star = \begin{bmatrix}
-0.3074545
\\\\
-0.3033940
\end{bmatrix}
$$

The solution, $u^\star$, is stored in vector `u`: it first stores the initial guess,
it is then passed by mutable reference to method `solve` and, on exit, it stores the 
solution, which is 
$$
u^\star = \begin{bmatrix}
-0.080608
\\\\
-0.090196
\\\\
-0.694680
\end{bmatrix}
$$

Lastly, we see that `delta_y_norm` is equal to `0.0000038`; this is equal to 
the norm-distance of $F_1(u)$ from $C$. We see that this is indeed below $\delta=10^{-5}$.

## Examples

See [`alm_pm.rs`](https://github.com/alphaville/optimization-engine/blob/master/examples/alm_pm.rs).

[`AlmOptimizer`]: https://docs.rs/optimization_engine/*/optimization_engine/alm/struct.AlmOptimizer.html
[`AlmFactory`]: https://docs.rs/optimization_engine/*/optimization_engine/alm/struct.AlmFactory.html
[Python interface]: ./python-interface
