---
id: faq
title: Frequently Asked Questions
sidebar_label: FAQ
---

### 1. Can it find the global optimum?
No. OpEn does not offer any guarantees that the solution it returns is a global optimum.

### 2. Can it solve mixed integer problems?
Yes. OpEn solves parametric problems of the form

```text
Minimize f(u; p)
subj. to: u in U(p)
```

Set `U(p)` can be 

### 3. Can it solve multi-objective problems?
No.

### 4. How should I choose the L-BFGS memory?
A value between 3 and 20 should suffice, although the performance is not too sensitive to the exact value of the L-BFGS memory. You shouldn't choose too high values as this will increase the memory requirements of your program (esp. for large-scale problems) and might have a negative effect on performance. 

Long story short... if you need very high performance, you should experiment with it. Otherwise, stick with any value between 3 and 20.

### 5. How can I cite OpEn?
Please, cite the original PANOC paper as follows:
```bibtex
@inconference{panoc2017,
  author = "Lorenzo Stella 
   			and Andreas Themelis 
   			and Pantelis Sopasakis 
   			and Panagiotis Patrinos",
  title     = {A simple and efficient algorithm for nonlinear model predictive control},
  booktitle = {56th IEEE Conference on Decision and Control (CDC)},
  year      = {2017},
  pages     = {1939-1944},
  doi       = {10.1109/CDC.2017.8263933},
  url       = {https://doi.org/10.1109/CDC.2017.8263933}
}
```
Cite the software as follows:


