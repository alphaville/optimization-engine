---
title: New Python OCP Module for Optimal Control and MPC
author: Pantelis Sopasakis
authorURL: https://github.com/alphaville
authorImageURL: https://avatars.githubusercontent.com/u/125415?v=4
---

![State trajectories generated with the Python OCP module](/optimization-engine/img/ocp-states.png)

OpEn now comes with a Python OCP module that facilitates the design of optimal control problems in a direct and intuitive way.

With the new module, you can describe the key ingredients of an optimal control problem from Python, including:

- stage and terminal costs
- system dynamics
- state and input constraints
- problem parameters and defaults

This makes it much easier to formulate nonlinear optimal control problems and model predictive control (MPC) schemes before generating an embedded optimizer with OpEn.

<!-- truncate -->

The new OCP workflow is documented in the Python OCP guide:

- [Getting started with the OCP module](/optimization-engine/docs/python-ocp-1)
- [OCP problem formulation](/optimization-engine/docs/python-ocp-2)
- [Building the optimizer](/optimization-engine/docs/python-ocp-3)
- [Running closed-loop simulations](/optimization-engine/docs/python-ocp-4)

If you want to try it right away, you can also run the Colab notebook:

- [Try the Python OCP module in Google Colab](https://colab.research.google.com/drive/17vbVUbqcah9seIg17aN6bW0-T15FWrBo?usp=sharing)

This functionality was introduced in `opengen` version `0.10.0a1`, and it opens the door to a smoother workflow for designing optimal controllers and MPC applications directly in Python.
