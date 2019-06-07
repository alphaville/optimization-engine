---
id: openrust-homotopy
title: Homotopy method
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

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

<div class="math">\[c(u, p) = 0,\]</div>

is satisfied.



### Sequences

### Penalty method

## Example