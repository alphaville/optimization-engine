---
id: openrust
title: OpEn Rust
---

<script type="text/javascript" src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

## Problem definition
**OpEn** can solve problems of the form:

<div class="math">
\[\begin{split}\operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{n_u}}&amp;\ \ f(u; p)\\
\mathrm{subject\ to} &amp;\ \ u \in U(p)\end{split}\]</div>

where *f* is a C1,1 function (continuously diff/ble with Lipschitz-continuous gradient) and *U* is a set on which we may project.

The definition of an optimization problem consists in specifying the following three componenets:

- the cost function *f* as a Rust function
- the gradient of *f*, *df*, as a Rust function
- the set of constraints, as an implementation of a trait

### Cost functions 
The **cost function** `f` is a Rust function of type `|u: &[f64], cost: &mut f64| -> i32`. The first argument, `u`, is the argument of the function. The second argument, is a mutable reference to the result (cost). The function returns an integer *status code*; the status code `0` means that the computation was successful. Nonzero status codes can be used to encode errors/exceptions.

As an example, consider the cost function `f` that maps a two-dimensional vector `u` to `f(u) = 5.0 * u[0] - u[1]^2`. This will be:


```rust
let f = |u: &[f64], c: &mut f64| -> i32 {
	*c = 5.0 * u[0] - u[1].powi(2);
    0
};
```

The **gradient of the cost** is a function `df` with signature `|u: &[f64], grad: &mut [f64]| -> i32`. The first argument, `u`, is again the argument of the function. The second argument, is a mutable reference to the result (gradient). The function returns again a status code (same as above). 

For the cost function `f(u) = 5.0 * u[0] - u[1]^2`, the gradient is given by `df(u) = [5.0, - 2.0*u[1]]`. This function can be implemented as follows:

```rust
let df = |u: &[f64], grad: &mut [f64]| -> i32 {
	grad[0] = 5.0;
	grad[1] = -2.0*u[1];
	0
};
```

### Constraints
Constraints implement the namesake trait, `Constraint`. Implementations of `constraint` implement the method `project` which computes projections on the set of constraints. This way, users can implement their own constraints. **OpEn** comes with the following implementations of `Constraint`:

- Euclidean balls (`Ball2`)
- Rectangles (`Rectangle`)
- No Constraints (`NoConstraints`)

These are the most common constraints in practice.

The construction of a constraint is very easy. Here is an example of a Euclidean ball centered at the origin with given radius:

```rust
let radius = 0.5;
let bounds = constraints::Ball2::new_at_origin_with_radius(radius);
```

### Problems
Having defined a cost function, its gradient and the constraints, we may define an optimization problem. Here is an example:

```rust
let problem = Problem::new(bounds, df, f);
```

Note that `problem` now owns the constraints, the gradient and the cost function.

## Calling the solver

**OpEn** uses three essential structures: (i) a cache, (ii) an engine and (iii) an optimizer. 

### Cache 
Rarely will one need to solve a *single* optimization problem in an engineering application. The solution of an optimization problem requires the allocation of memory. A **cache** allows for multiple instances of a problem to have a common workspace, so that we will not need to free and reallocate memory unnecessarily. A cache object will be reused once we need to solve a similar problem; this is the case with model predictive control, where an optimal control problem needs to be solved at every time instant.

```rust
let n = 50;
let lbfgs_memory = 10;
let tolerance = 1e-6;
let mut panoc_cache = PANOCCache::new(
    NonZeroUsize::new(n).unwrap(),
    tolerance,
    NonZeroUsize::new(lbfgs_memory).unwrap(),
);
```

### Engine
An **engine** holds all necessary information to execute a step of the algorithm. An engine always owns an instance of the problem and holds a (mutable) reference to a cache object. An engine is constructed easily as follows:


```rust
let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);
```

Instances of `Engine` typically need to be defined as mutable. An engine is then passed on to an Optimizer which solves the problem.


### Optimizer
The last necessary step is the construction of an Optimizer. An Optimizer uses an instance of Engine to run the algorithm and solve the optimization problem. An optimizer may have additional parameters such as the maximum number of iterations, which can be configured. Here is an example:

```rust
let mut panoc = PANOCOptimizer::new(&mut panoc_engine);
panoc.with_max_iter(max_iters);
```

We may then call the solver using the method `solve` and providing an initial guess:

```rust
let status = panoc.solve(&mut u);
```
This will return the solver status and will update `u` with the solution.


### Solver statistics
Method `solve` returns an object of type `SolverStatus` which stores information about the solver, namely, whether the required tolerance was attained (`has_converged()`), the number of iterations (`get_number_iterations()`), the norm of the residual (a measure of the accuracy of the solution) (`get_norm_fpr()`) and the cost value at the approximate solution (`get_cost_value()`).

## Complete Example in Rust
The minimization of the [Rosenbrock function](https://en.wikipedia.org/wiki/Rosenbrock_function) is a challenging problem in optimization. The Rosenbrock function in two dimensions with parameters *a* and *b* is defined as follows:

```rust
pub fn rosenbrock_cost(a: f64, b: f64, u: &[f64]) -> f64 {
    (a - u[0]).powi(2) + b * (u[1] - u[0].powi(2)).powi(2)
}

pub fn rosenbrock_grad(a: f64, b: f64, u: &[f64], grad: &mut [f64]) {
    grad[0] = 2.0 * u[0] - 2.0 * a - 4.0 * b * u[0] * (-u[0].powi(2) + u[1]);
    grad[1] = b * (-2.0 * u[0].powi(2) + 2.0 * u[1]);
}
```

Here, we minimize the above two-dimensional Rosenbrock function with parameters `a=1` and `b=200` subject to the constraint `|u| <= 1`, that is, that the decision variable *u* should be contained in a unit (Euclidean) ball.

The required tolerance is `1e-14`. The memory of the L-BFGS buffer is set to `10`; typically, a value between `3` and `20` will suffice.

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
	let df = |u: &[f64], grad: &mut [f64]| -> i32 {
	    rosenbrock_grad(a, b, u, grad);
	    0
	};
	let f = |u: &[f64], c: &mut f64| -> i32 {
	    *c = rosenbrock_cost(a, b, u);
	    0
	};

	// define the constraints
	let bounds = constraints::Ball2::new_at_origin_with_radius(radius);

	/* PROBLEM STATEMENT */
	let problem = Problem::new(bounds, df, f);
	let mut panoc_cache = PANOCCache::new(
	    NonZeroUsize::new(n).unwrap(),
	    tolerance,
	    NonZeroUsize::new(lbfgs_memory).unwrap(),
	);
	let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);
	let mut panoc = PANOCOptimizer::new(&mut panoc_engine);
	panoc.with_max_iter(max_iters);

	// Invoke the solver
	let status = panoc.solve(&mut u);
}
```

## Solving parametric problems

In embedded applications, we typically need to solve parametric problems, that is, problems of the form

```text
Minimize f(u,p)
subject to: u in U(p)
```

where `u` is the decision variable and `p` is a parameter.

As mentioned above, a **cache** needs to be constructed **once**. Then, every time the time of `p` changes, we define the cost function `g(u) = f(u, p)` and the set of constraints `U = U(p)`, that is, we redefine the **problem**.

Let us give a complete example. We consider the problem of minimizing the two-dimensional Rosenbrock function, `f(u, a, b)` with parameters `a` and `b`, subject to the constraints `|u| <= r`. The associated vector of parameters in this case is `p = (a, b, r)`.

In the following example, we solve 100 problems while modifying the parameters, `p`, every time. Note that the cache is created outside the main loop, only once, and it is not updated.

Note also that the initial guess of each problem is the solution of the previous problem.

```rust
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
	let mut panoc_cache = PANOCCache::new(
		NonZeroUsize::new(n).unwrap(),
		tolerance,
		NonZeroUsize::new(lbfgs_memory).unwrap(),
	);	

	let mut i = 0;
	while i < 100 {
		// update the values of `a`, `b` and `radius`
		b *= 1.01;
		a -= 1e-3;
		radius += 0.001;

		// update the function definitions (`f` and `df`)
		let df = |u: &[f64], grad: &mut [f64]| -> i32 {
			mocks::rosenbrock_grad(a, b, u, grad);
			0
		};
		let f = |u: &[f64], c: &mut f64| -> i32 {
			*c = mocks::rosenbrock_cost(a, b, u);
			0
		};

		// define the bounds at every iteration
		let bounds = constraints::Ball2::new_at_origin_with_radius(radius);

		// the problem definition is updated at every iteration
		let problem = Problem::new(bounds, df, f);

		// the engine is also updated at every iteration
		let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);

		// updated instance of the solver
		let mut panoc = PANOCOptimizer::new(&mut panoc_engine);
		panoc.with_max_iter(max_iters);

		let status = panoc.solve(&mut u);

		i += 1;

		// print useful information
		println!(
			"parameters: (a={:.4}, b={:.4}, r={:.4}), iters = {}",
			a, b, radius, status.get_number_iterations()
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