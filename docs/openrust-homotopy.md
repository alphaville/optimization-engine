---
id: openrust-homotopy
title: Homotopy method
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

![Rust Programming](/optimization-engine/img/rust1.jpeg)

## Problem statement

Consider the following parametric optimization problem:

<div class="math">
\[\begin{split}\mathbb{P}(p) {}:{} 
\operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{n_u}}&amp;\ \ f(u; p)\\\mathrm{subject\ to} &amp;\ \ u \in U\end{split}\]</div>

with parameter vector $p {}\in{} \mathbb{R}^{n_p}$. 

Let $\mathcal{I} \subseteq \\{1,2,\ldots, n_p\\}$
be a set of indices and let $p_i^*\in \mathbb{R} \cup \\{ \pm \infty \\}$. 

The homotopy method 
consists in constructing a sequence of values $(p_i^\nu)_{\nu}$ for each $i\in\mathcal{I}$ and solving
a sequence of optimization problems $\mathbb{P}(p^\nu)$, where $p^\nu_i = p_i$ for $i\notin \mathcal{I}$.

The procedure continues until $p^\nu_i = p_i^*$ for all $i\in\mathcal{I}$ (the target is reached for 
all $i$), or a termination criterion of the form 

<div class="math">\[\|c(u, p)\|_{\infty} \leq \epsilon_h,\]</div>

is satisfied.

The optimizer of each problem is passed on to the next one and used as an initial guess.



### Sequences

For the construction of the aforementioned sequences, the user needs to specify:

- the initial values of the parameters, $p^0$
- the final values, $p^*$, and
- the update rule

There are two main update rules:

- The [Geometric update], where $p^{\nu+1} = \alpha p^{\nu}$ and can only be used
  when the final value is zero (with $|\alpha| < 1$) or when it is $\infty$ (with $|\alpha|>1$)
- The [Convex update], where $p^{\nu+1}=\alpha{}p^{\nu}{}+{}(1-\alpha)p^{\ast}$, 
   which only makes sense when $p^{\ast}_i$ are finite.

### Penalty method
The penalty method can be employed to solve problems of the form

<div class="math">
\[\begin{split}\mathbb{P}(p) {}:{} 
\operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{n_u}}&amp;\ \ f(u; p)
\\\mathrm{subject\ to} &amp;\ \ u \in U
\\
&amp;\ \ c_i(u, p) = 0, i=1,\ldots, n_c\end{split}\]</div>

In that case, we define the modified cost function

<div class="math">\[F(u, p, \lambda) = f(u; p) + \sum_{i=1}^{n_c}\lambda_i g(c_i(u, p))\].</div>

We define the augmented vector of parameters $q = (p, \lambda)$ and construct
a sequence $q^\nu = (p, \lambda^\nu)$, where $\lambda_i^\nu \to \infty$ as 
$\nu \to \infty$.

Function $g$, the *penalty function*, is typically chosen to be $g(z) = z^2$.

In the penalty method, we construct a sequence of problems

<div class="math">
\[\begin{split}\mathbb{P}(p) {}:{} 
\operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{n_u}}&amp;\ \ F(u; p)\\\mathrm{subject\ to} &amp;\ \ u \in U\end{split}\]</div>

which can be solved as discussed in the [previous section].

## Example

Here is a simple example: Consider the problem of minimizing the two-dimensional 
Rosenbrock function 

<div class="math">\[f(u; a, b) = (a - u_1)^2 + b(u_2 - u_1^2)^2,\]</div>

subject to the constraints 

<div class="math">\[u \in U := \{u\in\mathbb{R}^2 : -1 \leq u_i \leq 1, i=1,2\}\]</div>

and subject to the additional constraints $1.5u_1 - u_2 = 0$. We first define $c(u, p) = 1.5u_1 - u_2$
and the modified cost function

<div class="math">\[F(u, a, b, \lambda) = f(u; a, b) + \lambda(1.5u_1 - u_2)^2,\]</div>

which has gradient

<div class="math">\[
    \begin{align}
    \nabla F(u; a, b, \lambda) =& \nabla f(u; a, b) + \lambda \begin{bmatrix}3(1.5u_1 - u_2)\\-2(1.5u_1 - u_2)\end{bmatrix}
    \\
    =&
    \begin{bmatrix}
2 (u_1-a) - 4bu_1(u_2 - u_1^2) + 3\lambda(1.5u_1 - u_2)
\\
2b(u_2 - u_1^2) -2\lambda(1.5u_1 - u_2)
\end{bmatrix}
    \end{align}\]</div>

In OpEn, we need to specify now three functions:

- the cost function, $F$
- the gradient of the cost function, $\nabla F$ and 
- the termination function, $c$ (here, the constraints)

The cost function and its gradient must be closures which follow the following signature:

```rust
/* cost function, f(u; q) */
let F_funct = |u: &[f64], q: &[f64], cost: &mut f64| -> Result<(), SolverError> {
    *cost = (q[0] - u[0]).powi(2)
        + q[1] * (u[1] - u[0].powi(2)).powi(2)
        + q[2] * (1.5 * u[1] - u[0]).powi(2);
    Ok(())
};

    /* parametric gradient, df(u, q) */
let dF_funct = |u: &[f64], q: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
    let t = 1.5 * u[0] - u[1];
    let s = u[1] - u[0].powi(2);
    grad[0] = 2.0 * (u[0] - q[0]) - 4.0 * q[1] * s + 3.0 * q[2] * t;
    grad[1] = 2.0 * q[1] * s - 2.0 * q[2] * t;
    Ok(())
};
```

and, similarly, the termination function must be a closure with signature

```rust
/* penalty-type constraints: c(u; p) */
let c_funct =
    |u: &[f64], _q: &[f64], constr: &mut [f64]| -> Result<(), SolverError> {
        constr[0] = 1.5 * u[0] - u[1];
        Ok(())
    };
```        

Next, we need to specify the constraints $u \in U$. This is 

```rust
// Constraints...
let xmin = &[-1.0, -1.0];
let xmax = &[1.0, 1.0];
let bounds = Rectangle::new(Some(xmin), Some(xmax));
```

We may now construct an instance of [`HomotopyProblem`] and provide the problem data

```rust
let num_constraints = 1;
let homotopy_problem = continuation::HomotopyProblem::new(
    bounds,
    gradient_function,
    cost_function,
    penalty_constr_function,
    num_constraints,
);
```

The last piece of information we need to provide before we can solve the problem is the 
continuation type. Recall that we have constructed a parametric problem with parameter
vector $p = (a, b, \lambda)$ and we need to drive $p_3 = \lambda$  from an initial value 
$p_3^0$ to $+\infty$. The following code snippet is self-explanatory:

```rust
homotopy_cache.add_continuation(
    2,                           // drive q[2] (this is q_3)
    100.,                        // from the initial value: 100.0
    std::f64::INFINITY,          // to infinity
    Geometric(10.0),             // by multiplying by 10.0
);
```

The solver will produce a sequence of parameters $q^\nu$, $\nu=0,1,\ldots$, with 
$q_3^{\nu + 1} = 10 q^\nu_3$ and $q^0_3 = 100$ (that is, the sequence is $10^2, 10^3, 10^4, \ldots$).

We may now solve the problem...

```rust
let mut homotopy_optimizer =
    continuation::HomotopyOptimizer::new(homotopy_problem, &mut homotopy_cache)
    .with_constraint_tolerance(1e-4);
let mut u: [f64; 2] = [0.5, 0.8];
let p: [f64; 3] = [1., 1., 1.];
let status = homotopy_optimizer.solve(&mut u, &p);
```

The status (`status`) and the optimizer (`u`) of this problem are 

```text
status = Ok(
    HomotopySolverStatus {
        exit_status: Converged,
        num_outer_iterations: 5,
        num_inner_iterations: 641,
        last_problem_norm_fpr: 0.000000022720347668995174,
        max_constraint_violation: 0.0000001096374043774162,
        solve_time: 579.792µs
    }
)

u* = [0.6666109704346171, 0.9999163460145214]
```

The above example is available in the examples folder (see [`examples/homotopy.rs`]). You can run it
with 

```shell
$ cargo run --example homotopy
```

<!-- links -->


[Convex update]: https://docs.rs/optimization_engine/*/optimization_engine/continuation/enum.ContinuationMode.html
[Geometric update]: https://docs.rs/optimization_engine/*/optimization_engine/continuation/enum.ContinuationMode.html
[previous section]: /optimization-engine/docs/openrust-basic
[`HomotopyProblem`]: https://docs.rs/optimization_engine/*/optimization_engine/continuation/struct.HomotopyProblem.html
[`examples/homotopy.rs`]: https://github.com/alphaville/optimization-engine/blob/master/examples/homotopy.rs
