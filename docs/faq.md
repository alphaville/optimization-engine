---
id: faq
title: Frequently Asked Questions
sidebar_label: FAQ
---

### 0. Do I need to know Rust?

No, you are not expected to write any Rust code.

You can use OpEn through its [Python](./python-interface) or [MATLAB](./matlab-interface) interface, where you will be able to define your parametric optimization problem and generated an optimizer. The auto-generated optimizer will be in Rust - you just need to [install the Rust compiler](https://www.rust-lang.org/tools/install), but you will not be required to use the compiler directly (the Python/MATLAB interface will do that for you).

### 1. Can OpEn find the global optimum?
No. 

OpEn does not offer any guarantees that the solution it returns is a global optimum.

### 2. Can OpEn solve mixed integer problems?
Yes. OpEn solves parametric problems of the form

```text
Minimize f(u; p)
subj. to: u in U(p)
```

Set `U(p)` can be a finite or binary set (on which it is easy to project).

### 3. Can OpEn solve multi-objective problems?
No.

### 4. How should I choose the L-BFGS memory?
A value between 3 and 20 should suffice, although the performance is not too sensitive to the exact value of the L-BFGS memory. You shouldn't choose too high values as this will increase the memory requirements of your program (esp. for large-scale problems) and might have a negative effect on performance. 

Long story short... if you need very high performance, you should experiment with it. Otherwise, stick with any value between 3 and 20.

### 5. How can I cite OpEn?
Please, cite the original PANOC paper as follows:
```bibtex
@inconference{panoc2017,
  author = "Stella, L. and Themelis, A. and Sopasakis, P. and Patrinos, P.",
  title     = "A simple and efficient algorithm for
               nonlinear model predictive control",
  booktitle = "56th IEEE Conference on Decision and Control (CDC)",
  year      = "2017",
  pages     = "1939-1944"
}
```

You may also cite the following papers where we use PANOC in lab experiments - first, for obstacle avoidance on an autonomous ground vehicle carrying a trailer:

```bibtex
@inconference{agv2018,
  author = "Sathya, A. and Sopasakis, P. and Van Parys, R. and Themelis, A. 
            and Pipeleers, G. and Patrinos, P.",
  title     = "Embedded nonlinear model predictive control for 
               obstacle avoidance using PANOC",
  booktitle = "IEEE European Control Conference (ECC)",
  year      = "2018",
}
```

and our recent work on obstacle avoidance of an autonomous micro-aerial vehicle:

```bibtex
@inconference{mav2019,
  author = "Small, E. and Sopasakis, P. and Fresk, E.
        and Patrinos, P. and Nikolakopoulos, G.",
  title     = "Aerial navigation in obstructed environments with
               embedded nonlinear model predictive control",
  booktitle = "IEEE European Control Conference (ECC)",
  year      = "2019"
}
```



### 6. I have a question; whom should I address it to?
You can reach us on Discord and Gitter. 

Find out more information [here](https://alphaville.github.io/optimization-engine/blog/2019/03/06/talk-to-us).

### 7. How can I report an issue?
Please report any issues on [github](https://github.com/alphaville/optimization-engine/issues).

