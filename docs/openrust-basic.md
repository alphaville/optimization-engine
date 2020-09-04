---
id: openrust-basic
title: Basic usage
description: How to use OpEn directly in Rust
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.1/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

![Rust Programming](/optimization-engine/img/rust1.jpeg)

## Problem definition
**OpEn** can solve problems of the form:

<div class="math">
\[\begin{split}\operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{n_u}}&amp;\ \ f(u, p)\\
\mathrm{subject\ to} &amp;\ \ u \in U(p)\end{split}\]</div>

where $f$ is a $C^{1,1}$ function (continuously diff/ble with Lipschitz-continuous gradient) and $U$ is a set on which we may project.

The definition of an optimization problem consists in specifying the following three componenets:

- the cost function $f$ as a Rust function
- the gradient of $f$, $\nabla f$, as a Rust function
- the set of constraints, $U$, as an implementation of a trait

### Cost functions
The **cost function** `f` is a Rust function of type `|u: &[f64], cost: &mut f64| -> Result<(), SolverError>`. The first argument, `u`, is the argument of the function. The second argument, is a mutable reference to the result (cost). The function returns a *status code* of the type `Result<(), SolverError>` and the status code `Ok(())` means that the computation was successful. Other status codes can be used to encode errors/exceptions as defined in the [`SolverError`] enum.

As an example, consider the cost function $f:\mathbb{R}^2\to\mathbb{R}$ that maps a two-dimensional 
vector $u$ to $f(u) = 5 u_1 - u_2^2$. This will be:


```rust
let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
	*c = 5.0 * u[0] - u[1].powi(2);
    Ok(())
};
```

The **gradient of the cost** is a function `df` with signature `|u: &[f64], grad: &mut [f64]| -> Result<(), SolverError>`. The first argument, `u`, is again the argument of the function. The second argument, is a mutable reference to the result (gradient). The function returns again a status code (same as above).

For the cost function $f(u) = 5 u_1 - u_2^2$, the gradient is given by 

<div class="math">
\[\nabla{}f(u) = \begin{bmatrix}5\\-2u_2\end{bmatrix}
\]</div>

This function can be implemented as follows:

```rust
let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
	grad[0] = 5.0;
	grad[1] = -2.0*u[1];
	Ok(())
};
```

### Constraints
Constraints implement the namesake trait, [`Constraint`]. Implementations of [`Constraint`] implement the method `project` which computes projections on the set of constraints. This way, users can implement their own constraints. **OpEn** comes with the following implementations of [`Constraint`]:


| Constraint           | Explanation                                          |
|----------------------|------------------------------------------------------|
| [`Ball2`]            | $U {}={} \\{u\in\mathbb{R}^n : \Vert u-u^0\Vert_2 \leq r\\}$         |
| [`BallInf`]          | $U {}={} \\{u\in\mathbb{R}^n : \Vert u-u^0\Vert_\infty \leq r\\}$         |
| [`Halfspace`]        | $U {}={} \\{u\in\mathbb{R}^n : \langle c, u\rangle \leq b\\}$ |
| [`Hyperplane`]       | $U {}={} \\{u\in\mathbb{R}^n : \langle c, u\rangle {}={} b\\}$ |
| [`Rectangle`]        | $U {}={} \\{u\in\mathbb{R}^n : u_{\min} \leq u \leq u_{\max}\\}$ |
| [`NoConstraints`]    | $U {}={} \mathbb{R}^n$                                   |
| [`FiniteSet`]        | $U {}={} \\{u^{(1)}, u^{(2)},\ldots,u^{(N)}\\}$          |
| [`SecondOrderCone`]  | $U {}={} \\{u=(z,t), t\in\mathbb{R}, \Vert{}z{}\Vert \leq \alpha t\\}$ |
| [`Zero`]             | $U {}={} \\{0\\}$                                        |
| [`CartesianProduct`] | Cartesian products of any of the above               |

These are the most common constraints in practice.

The construction of a constraint is very easy. Here is an example of a Euclidean ball centered at the origin with given radius:

```rust
let radius = 0.5;
let bounds = constraints::Ball2::new(None, radius);
```

### Problems
Having defined a cost function, its gradient and the constraints, we may define an optimization problem. Here is an example:

```rust
let problem = Problem::new(&bounds, df, f);
```

Note that [`problem`] now owns the gradient and the cost function
and borrows the constraints.

## Calling the solver

**OpEn** uses two essential structures: (i) a **cache** and (ii) an **optimizer**.

### Cache
Rarely will one need to solve a *single* optimization problem in an embedded engineering application. 
The solution of an optimization problem requires the allocation of memory. 
A **cache** (see [`PANOCCache`]) allows for multiple instances of a problem to have a common workspace, so that we will
not need to free and reallocate memory unnecessarily. 
A cache object will be reused once we need to solve a similar problem; this is the case with model 
predictive control, where an optimal control problem needs to be solved at every time instant.

```rust
let n = 50;
let lbfgs_memory = 10;
let tolerance = 1e-6;
let mut panoc_cache = PANOCCache::new(n, tolerance, lbfgs_memory);
```

### Optimizer
The last necessary step is the construction of an [`Optimizer`]. An Optimizer uses an instance of the problem adn the cache to run the algorithm and solve the optimization problem. An optimizer may have additional parameters such as the maximum number of iterations, which can be configured. Here is an example:

```rust
let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache)
    .with_max_iter(max_iters);
```

We may then call the solver using the method [`solve`] providing an initial guess:

```rust
let status = panoc.solve(&mut u).unwrap();
```

This will return the solver status (if it is successful) and will update `u` with the solution.

**Note:** The algorithm may, in general, return an error (instance of [`SolverError`]), this is
why the caller function should not use `unwrap` directly, but should rather check whether the
solver returned an error. The error can be propagated upstream using `let status = panoc.solve(&mut u)?;`.


### Solver statistics
Method `solve`, if successful, returns an object of type [`SolverStatus`] which stores information about the solver, namely, whether the required tolerance was attained (`has_converged()`), the [exit status] (`exit_status()`), the number of iterations (`iterations()`), the norm of the residual (a measure of the accuracy of the solution) (`norm_fpr()`) and the cost value at the approximate solution (`cost_value()`).

## Complete Example in Rust
The minimization of the [Rosenbrock function](https://en.wikipedia.org/wiki/Rosenbrock_function) 
is a challenging problem in optimization. 
The Rosenbrock function in two dimensions with parameters $a$ and $b$ is defined as follows:

<div class="math">\[f(u, a, b) = (a - u_1)^2 + b(u_2 - u_1^2)^2,\]</div>

with gradient

<div class="math">\[\nabla f(u, a, b) = \begin{bmatrix}
2 (u_1-a) - 4bu_1(u_2 - u_1^2)
\\
2b(u_2 - u_1^2)
\end{bmatrix},\]</div>

that is,

```rust
pub fn rosenbrock_cost(a: f64, b: f64, u: &[f64]) -> f64 {
    (a - u[0]).powi(2) + b * (u[1] - u[0].powi(2)).powi(2)
}

pub fn rosenbrock_grad(a: f64, b: f64, u: &[f64], grad: &mut [f64]) {
    grad[0] = 2.0 * (u[0] - a) - 4.0 * b * u[0] * (-u[0].powi(2) + u[1]);
    grad[1] = 2.0 * b * (u[1] - u[0].powi(2));
}
```

Here, we minimize the above two-dimensional Rosenbrock function with parameters $a=1$
and $b=200$ subject to the constraint $\|u\| {}\leq{} 1$, that is, that the decision variable 
$u$ should be contained in a unit (Euclidean) ball.

The required tolerance is $10^{-14}$. The memory of the L-BFGS buffer is set to 10; 
typically, a value between 3 and 20 will suffice.

```rust
fn main() {
	/* USER PARAMETERS */
	let tolerance = 1e-14;
	let a = 1.0;
	let b = 200.0;
	let n = 2;
	let lbfgs_memory = 10;
	let max_iters = 80;
	let mut u = [-1.5, 0.9];
	let radius = 1.0;

	// define the cost function and its gradient
	let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
	    rosenbrock_grad(a, b, u, grad);
	    Ok(())
	};
	let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
	    *c = rosenbrock_cost(a, b, u);
	    Ok(())
	};

	// define the constraints
	let bounds = constraints::Ball2::new(None, radius);

	/* PROBLEM STATEMENT */
	let problem = Problem::new(&bounds, df, f);
	let mut panoc_cache = PANOCCache::new(n, tolerance, lbfgs_memory);
	let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache)
			.with_max_iter(max_iters);

	// Invoke the solver
	let status = panoc.solve(&mut u);
}
```

This example can be found in [`examples/panoc_ex1.rs`](https://github.com/alphaville/optimization-engine/blob/master/examples/panoc_ex1.rs).

## Solving parametric problems

In embedded applications, we typically need to solve parametric problems, that is, problems of the form

<div class="math">
\[\begin{split}\operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{n_u}}&amp;\ \ f(u, p)\\
\mathrm{subject\ to} &amp;\ \ u \in U(p)\end{split}\]</div>

where $u$ is the decision variable and $p$ is a parameter.

As mentioned above, a **cache** needs to be constructed **once**. 

Then, every time the value of $p$ changes, we define the cost function $g(u) = f(u, p)$ 
and the set of constraints $U = U(p)$, that is, we redefine the **problem**.

Let us give a complete example. We consider the problem of minimizing the two-dimensional 
Rosenbrock function, $f(u, a, b)$ with parameters $a$ and $b$, subject to the constraints 
$\|u\| {}\leq{} r$. The associated vector of parameters in this case is $p = (a, b, r)$.

In the following example, we solve 100 problems while modifying the parameters, $p$, 
every time. Note that the cache is created outside the main loop, only once, and it is not
updated.

Note also that the initial guess of each problem is the solution of the previous problem.

```rust
use optimization_engine::{constraints::*, panoc::*, *};

fn rosenbrock_cost(a: f64, b: f64, u: &[f64]) -> f64 {
    (a - u[0]).powi(2) + b * (u[1] - u[0].powi(2)).powi(2)
}

fn rosenbrock_grad(a: f64, b: f64, u: &[f64], grad: &mut [f64]) {
    grad[0] = 2.0 * u[0] - 2.0 * a - 4.0 * b * u[0] * (-u[0].powi(2) + u[1]);
    grad[1] = b * (-2.0 * u[0].powi(2) + 2.0 * u[1]);
}

fn main() {
	let tolerance = 1e-6;
	let mut a = 1.0;
	let mut b = 100.0;
	let n = 2;
	let lbfgs_memory = 10;
	let max_iters = 100;
	let mut u = [-1.5, 0.9];
	let mut radius = 1.0;

	// the cache is created only ONCE
	let mut panoc_cache = PANOCCache::new(n, tolerance, lbfgs_memory);

	let mut i = 0;
	while i < 100 {
		// update the values of `a`, `b` and `radius`
		b *= 1.01;
		a -= 1e-3;
		radius += 0.001;

		// define the cost function and its gradient
		let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
			if a < 0.0 || b < 0.0 {
				Err(SolverError::Cost)
			} else {
				rosenbrock_grad(a, b, u, grad);
				Ok(())
			}
		};

		let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
			if a < 0.0 || b < 0.0 {
				Err(SolverError::Cost)
			} else {
				*c = rosenbrock_cost(a, b, u);
				Ok(())
			}
		};		

	    // define the bounds at every iteration
	    let bounds = constraints::Ball2::new(None, radius);
	
		// the problem definition is updated at every iteration
		let problem = Problem::new(&bounds, df, f);

		// updated instance of the solver
		let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache)
				.with_max_iter(max_iters);

		let status = panoc.solve(&mut u).unwrap();

		i += 1;

		// print useful information
		println!(
			"parameters: (a={:.4}, b={:.4}, r={:.4}), iters = {}",
			a, b, radius, status.iterations()
		);
		println!("u = {:#.6?}", u);
	}
}
```

The above function will print

```text
parameters: (a=0.9990, b=101.0000, r=1.0010), iters = 43
u = [
    0.786980,
    0.618599
]
parameters: (a=0.9980, b=102.0100, r=1.0020), iters = 5
u = [
    0.787543,
    0.619500
]
parameters: (a=0.9970, b=103.0301, r=1.0030), iters = 5
u = [
    0.788107,
    0.620400
]
parameters: (a=0.9960, b=104.0604, r=1.0040), iters = 5
u = [
    0.788670,
    0.621301
]
...
```

## Real time implementation
In a real time implementation, it is important for our parametric optimizer to meet the 
maximum runtime requirements. For example, in digital optimization-based control or estimation
applications, the computation needs to complete well within the sampling period of the 
system.

OpEn provides the method [`with_max_duration`] in [`PANOCOPtimizer`] that allows us to set 
a maximum time duration, after which the solver stops. If it fails to converge because of 
the imposition of a maximum allowed duration, the exit status will be 
[`ExitStatus::NotConvergedOutOfTime`].

## Examples

- [`panoc_ex1.rs`](https://github.com/alphaville/optimization-engine/blob/master/examples/panoc_ex1.rs)
- [`panoc_ex2.rs`](https://github.com/alphaville/optimization-engine/blob/master/examples/panoc_ex2.rs)

<!-- Links -->

[`Constraint`]: https://docs.rs/optimization_engine/*/optimization_engine/constraints/trait.Constraint.html
[`Ball2`]: https://docs.rs/optimization_engine/*/optimization_engine/constraints/struct.Ball2.html
[`BallInf`]: https://docs.rs/optimization_engine/*/optimization_engine/constraints/struct.BallInf.html
[`Halfspace`]: https://docs.rs/optimization_engine/*/optimization_engine/constraints/struct.Halfspace.html
[`Hyperplane`]: https://docs.rs/optimization_engine/*/optimization_engine/constraints/struct.Hyperplane.html
[`Zero`]: https://docs.rs/optimization_engine/*/optimization_engine/constraints/struct.Zero.html
[`FiniteSet`]: https://docs.rs/optimization_engine/*/optimization_engine/constraints/struct.FiniteSet.html
[`CartesianProduct`]: https://docs.rs/optimization_engine/*/optimization_engine/constraints/struct.CartesianProduct.html
[`SecondOrderCone`]: https://docs.rs/optimization_engine/*/optimization_engine/constraints/struct.SecondOrderCone.html
[`Rectangle`]: https://docs.rs/optimization_engine/*/optimization_engine/constraints/struct.Rectangle.html
[`NoConstraints`]: https://docs.rs/optimization_engine/*/optimization_engine/constraints/struct.NoConstraints.html
[`SolverError`]: https://docs.rs/optimization_engine/*/optimization_engine/enum.SolverError.html
[`problem`]: https://docs.rs/optimization_engine/*/optimization_engine/core/problem/struct.Problem.html
[`PANOCCache`]: https://docs.rs/optimization_engine/*/optimization_engine/core/panoc/struct.PANOCCache.html
[`Optimizer`]: https://docs.rs/optimization_engine/*/optimization_engine/core/trait.Optimizer.html
[`SolverStatus`]: https://docs.rs/optimization_engine/*/optimization_engine/core/solver_status/struct.SolverStatus.html
[`PANOCOPtimizer`]: https://docs.rs/optimization_engine/*/optimization_engine/core/panoc/struct.PANOCOptimizer.html
[`with_max_duration`]: https://docs.rs/optimization_engine/*/optimization_engine/core/panoc/struct.PANOCOptimizer.html#method.with_max_duration
[exit status]: https://docs.rs/optimization_engine/*/optimization_engine/core/enum.ExitStatus.html
[`ExitStatus::NotConvergedOutOfTime`]: https://docs.rs/optimization_engine/*/optimization_engine/core/enum.ExitStatus.html#variant.NotConvergedOutOfTime
[`solve`]: https://docs.rs/optimization_engine/*/optimization_engine/core/trait.Optimizer.html#tymethod.solve
