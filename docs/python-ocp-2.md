---
id: python-ocp-2
title: Constructing OCPs
description: Optimal Control with OpEn/opengen
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.1/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>
<style>
.but{
  border: none;
  color: #348c4f;
  padding: 15px 20px;
  text-align: center;
  text-decoration: none;
  display: inline-block;
  font-size: 16px;
  margin: 0px 0px;
  cursor: pointer;
  width: 250px;
  border-radius: 8px;
}
</style>
<style>
.but1 {
    background-color: #e9e642;
}
</style><style>
.but2 {
    background-color: #008CBA;
}
</style>

<div class="alert alert-warning">
<b>Info:</b> The functionality presented here was introduced in <code>opengen</code> version <code>0.10.0a1</code>.
The API is still young and is likely to change in version <code>0.11</code>.
</div>

Here we will look at how we can construct an optimal control problem (OCP)
by defining its state and terminal cost functions, input and state 
constraints, prediction horizon and other options.
