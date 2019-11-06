---
id: faq
title: Frequently Asked Questions
sidebar_label: FAQ
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

<script>
  ((window.gitter = {}).chat = {}).options = {
    room: 'alphaville/optimization-engine'
  };
</script>
<script src="https://sidecar.gitter.im/dist/sidecar.v1.js" async defer></script>

### 1. Do I need to be able to program in Rust?

No. Although the core solver is written in Rust, you can
generate code in [Python](./python-interface) or
 [MATLAB](./matlab-interface) and use it from Python.
You can also use the auto-generated C/C++ bindings to use 
it in your embedded project.

Even if you are working in a Rust project, it is more 
convenient to generate Rust code using OpEn's Python
interface and then use the auto-generated code in your
project.

If your project is in some different language, you can 
either use the C/C++ bindings (if possible), or call
the solver over a TCP socket (supported by all modern
programming languages).
  
### 2. Can OpEn find the global optimum?
No. OpEn does not offer any guarantees that the solution it returns is a global optimum.

### 3. Can OpEn solve mixed integer problems?
Yes. OpEn solves parametric problems of the form

<div class="math">
\[\begin{split}\mathbb{P}(p) {}:{} \operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{n_u}}&amp;\ \ f(u, p)\\
\mathrm{subject\ to}\ \  &amp;u \in U\\
&amp; F_1(u, p) \in C\\
&amp; F_2(u, p) = 0\end{split}\]</div>

Set $U$ can be a finite or binary set (on which it is easy to compute projections).

### 4. Can OpEn solve multi-objective problems?
No.

### 5. How should I choose the L-BFGS memory?
A value between 3 and 20 should suffice, although the performance is not too sensitive to the exact value of the L-BFGS memory. You shouldn't choose too high values as this will increase the memory requirements of your program (esp. for large-scale problems) and might have a negative effect on performance. 

Long story short... if you need very high performance, you should experiment with it. Otherwise, stick with any value between 3 and 20.

### 6. How can I cite OpEn?
Please, cite the original PANOC paper as follows:
```bibtex
@inproceedings{panoc2017,
  author = "Stella, L. and Themelis, A. and Sopasakis, P. and Patrinos, P.",
  title     = "A simple and efficient algorithm for
               nonlinear model predictive control",
  booktitle = "56th IEEE Conference on Decision and Control (CDC)",
  year      = "2017",
  month     = "Dec",
  pages     = "1939-1944",
  doi       = "10.1109/CDC.2017.8263933"
}
```

You may also cite the following papers where we use PANOC in lab experiments - first, for obstacle avoidance on an autonomous ground vehicle carrying a trailer:

```bibtex
@inproceedings{agv2018,
  author = "Sathya, A. and Sopasakis, P. and Van Parys, R. and Themelis, A. 
            and Pipeleers, G. and Patrinos, P.",
  title     = "Embedded nonlinear model predictive control for 
               obstacle avoidance using PANOC",
  booktitle = "IEEE European Control Conference (ECC)",
  year      = "2018",
  month     = "June",
  pages     = "1523-1528",
  doi       = "10.23919/ECC.2018.8550253"
}
```

and our recent work on obstacle avoidance of an autonomous micro-aerial vehicle:

```bibtex
@inproceedings{mav2019,
  author = "Small, E. and Sopasakis, P. and Fresk, E.
        and Patrinos, P. and Nikolakopoulos, G.",
  title     = "Aerial navigation in obstructed environments with
               embedded nonlinear model predictive control",
  booktitle = "IEEE European Control Conference (ECC)",
  year      = "2019",
  month     = "June",
  pages     = "3556-3563",
  doi       = "10.23919/ECC.2019.8796236"
}
```

### 7. Does it work on all operating systems?
Yes. We have tested the code on Linux (Trusty, Precise 
and Xenial), OSX Darwin and Windows x32 adn x64
using Travis CI and Appveyor reprectively. We have 
also cross-compiled the Rust solver and tested it 
on a Raspberry Pi v2 (ARM v6 processor).

### 8. I have a question; whom should I address it to?
You can reach us on Discord and Gitter. 

Find out more information [here](https://alphaville.github.io/optimization-engine/blog/2019/03/06/talk-to-us).

### 9. How can I report an issue?
Please report any issues on [github](https://github.com/alphaville/optimization-engine/issues).

