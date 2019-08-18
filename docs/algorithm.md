---
id: algorithm
title: Algorithm
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>



Optimization Engine (OpEn) solves problems of the following form:

<div class="math">
\[\begin{split}\operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{n_u}}&amp;\ \ f(u; p)\\
\mathrm{subject\ to} &amp;\ \ u \in U\\
&amp;\ \ c(u; p) = 0\end{split}\]</div>

Our blanket assumptions are:

**A1.** Function $f({}\cdot{}; p):\mathbb{R}^n\to\mathbb{R}$ is continuously differentiable in $u$ with $L$-Lipschitz
   gradient

**A2.** $U\subseteq \mathbb{R}^n$ is a nonempty set on which we can compute projections (e.g., boxes, balls, discrete sets, etc)

**A3.** Functions $c_i({}\cdot{}; p):\mathbb{R}^n\to\mathbb{R}$, for $i=1,\ldots, s$, are continuous

We shall first assume that $s=0$, i.e., there are no constraints of the form $c(u; p) {}={} 0$.

## Simple constraints

Let us first focus on problems of the following simpler form (we have dropped the dependence 
on $p$ for the sake of brevity)

<div class="math">
\[\begin{split}\operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{n_u}}&amp;\ \ f(u)\\
\mathrm{subject\ to} &amp;\ \ u \in U\end{split}\]</div>

### Projected gradient method
Recall that the projected gradient method applied on this problem yields the iterations

<div class="math">
\[u^{\nu+1} = \Pi_{U}(u^\nu - \gamma \nabla f(u^\nu))\]</div>

starting from an initial guess $u^0$. 

Let us define $T_\gamma(u) = \Pi_U (u - \gamma \nabla f(u))$, so the projected gradient method
can be written as $u^{\nu+1} = T_\gamma(u^\nu)$, that is, they satisfy

<div class="math">\[u^{\star} = T_\gamma(u^\star)\]</div>

We call such points $u^\star$, $\gamma$-**critical points** of the problem. 

If $\gamma < 2/L$, then the projected gradient method is guaranteed to converge in the sense
that all accumulation points of the projected gradient method are fixed points of $T_\gamma$.

The projected gradient method involves only simple steps which makes it easy to
program and suitable for embedded applications, but convergence is at most Q-linear,
often slow and sensitive to ill conditioning.




### Speeding things up

In order to achieve superior congerence rates we proceed as follows:
a $\gamma$-crtical point of the above problem is a *zero* of the residual operator

<div class="math">\[R(u) = \tfrac{1}{\gamma}(u - T_{\gamma}(u))\]</div>

We may, therefore, use a Newton-type method of the form

<div class="math">\[u^{\nu+1} = u^\nu + d^\nu,\]</div>

where $d^\nu$ is computed as 

<div class="math">\[d^\nu = -H_\nu R_\gamma(u^\nu),\]</div>

where $H_\nu$ are invertible linear operators that satisfy the secant condition 
$s^\nu = H_\nu y^\nu$ with $s^\nu = u^{\nu+1}-u^\nu$ and 
$y^\nu = R_\gamma(u^{\nu+1})-R_\gamma(u^{\nu})$.

We could then use fast L-BFGS quasi-Newtonian directions, which do not require 
matrix operations (only vector products) and have low memory requirements.
However, convergence is only local, in a neighbourhood of a critical point $u^\star$.

Our objective is to employ a globalisation technique using an appropriate exact 
merit function.



### Forward-Backward Envelope

The forward-backward envelope (FBE) is a function $\varphi_\gamma:\mathbb{R}^n\to\mathbb{R}$
is a function with the following properties

1. It is real-valued
2. It is continuous
3. For $0 < \gamma < \tfrac{1}{L}$ it shares the same local/strong minima with the original problem

Therefore, the original constrained optimisation problem is equivalent to the unconstrained 
minimisation problem of function $\varphi_\gamma$.

Additionally, if $f\in C^2$, then FBE is $C^1$ with gradient $\nabla \varphi_\gamma(u) = (I-\gamma \nabla^2 f(u))R_\gamma(u)$.

The FBE is constructed as follows: Function $f$ can be approximated at a point $u \in U$ by the (local) quadratic 
upper bound:

<div class="math">\[Q^f_\gamma(v; u) = f(u) + \nabla f(u)^\top (v-u) + \frac{1}{2\gamma}\|v-u\|_2^2.\]</div>

Then, the FBE is defined as 

<div class="math">\[\varphi_\gamma(u) {}={} \inf_{v \in U}Q^f_\gamma(v; u).\]</div>

This construction is illustrated in the figure below

<img src="/optimization-engine/img/fbe.png" alt="FBE construction" width="500"/>

Provided that it is easy to compute the distance to $U$, the FBE can be easily computed by

<div class="math">\[\varphi_\gamma(u) {}={} f(u) - \tfrac{\gamma}{2}\|\nabla f(u)\|^2 + \mathrm{dist}^2(u - \gamma \nabla f(u)).\]</div>



### PANOC

The proximal averaged Newton-type method (PANOC) takes averaged iterated that combine
*safe* projected gradient updates and *fast* quasi-Newtonian directions, that is 

<div class="math">\[u^{\nu+1} = u^\nu + \tau_\nu\, \color{green}{d^\nu} + (1-\tau_\nu)\color{blue}{\gamma R_\gamma(u^\nu)}\]</div>

and $\tau_\nu$ is chosen so that 

<div class="math">\[\varphi_\gamma(u^{\nu+1}) \leq \varphi_\gamma(u^\nu) - \sigma \|R_\gamma(u^\nu)\|^2,\]</div>

for an appropriately chosen positive constant $\sigma$. You mean find more information about the algorithm
in [this paper](https://arxiv.org/abs/1904.10546).

Some facts about PANOC:

1. It uses the same oracle as the projected gradient method
2. Eventually only fast updates are activated: $u^{\nu+1} = u^\nu + d^\nu$
3. The sequence $r^\nu$ goes to 0 sq. summably
4. The cluster points of $u^\nu$ and $\bar{u}^\nu$ are $\gamma$-critical points
5. No need to solve any linear systems
6. Converges globally
7. and it is very fast in practice



## General constraints

Let us now move on to problems with more general constraints of the general form

<div class="math">
\[\begin{split}\operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{n_u}}&amp;\ \ f(u)\\
\mathrm{subject\ to} &amp;\ \ u \in U\\
&amp;\ \ c(u) = 0\end{split}\]</div>

Equality constraints of the form $c(u) = 0$ can be used to accommodate inequality ones
of the form

<div class="math">\[\psi(u) \leq 0,\]</div>

by taking $c(u) = \max\\{0, \psi(u)\\}$.

Although PANOC cannot solve the above problem directly, it can solve the following relaxed problem

<div class="math">
\[\begin{split}\mathbb{P}(\lambda) {}:{} \operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{n_u}}&amp;\ \ f(u) + \lambda g(c(u))\\
\mathrm{subject\ to} &amp;\ \ u \in U\end{split}\]</div>

where $g$ is a **penalty function**, which needs to be chosen so that $g(c(u))$ is a continuously
differentiable function with Lipschitz gradient. A typical (but not the only) choice for $g$
is $g(v) = v^2$ (the user is free to choose any penalty function).

Essentially, starting from an initial $\lambda_0>1$, we solve $\mathbb{P}(\lambda_\nu)$,
we update $\lambda_{\nu+1} = \beta \lambda_nu$, for some $\beta > 1$, and we then solve
$\mathbb{P}(\lambda_{\nu+1})$ using the previous optimal solution, $u_\nu^\star$, as an 
initial guess.

This procedure stops once $\\|c(u_\nu^\star)\\|_\infty {}<{} \epsilon_p$ for some 
tolerance $\epsilon_p > 0$. Tolerance $\epsilon_p$ controls the infeasibility of the 
solution.

Typically, few such outer iterations are required to achive a reasonable satisfaction of 
the constraints, which inner problems are solved very fast using PANOC.
