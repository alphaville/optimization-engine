import time
import casadi.casadi as cs
import opengen as og
import os
import sys
import numpy as np

#
# Before you run this, run
# python prepare_benchmarks.py
#
# Run benchmark with:
# py.test test/benchmark_open.py --benchmark-warmup-iterations=20
#
# To generate a histogram, do:
# py.test test/benchmark_open.py --benchmark-warmup-iterations=20 --benchmark-histogram=out
# qlmanage -t -s 1000 -o . out.svg
#

TEST_DIR = ".python_test_build/benchmarkable"

sys.path.insert(1, os.path.join(TEST_DIR, "benchmark1"))
sys.path.insert(1, os.path.join(TEST_DIR, "benchmark1p"))

import benchmark1
import benchmark1p

solver1 = benchmark1.solver()
solver1p = benchmark1p.solver()


def get_open_local_absolute_path():
    cwd = os.getcwd()
    return cwd.split('open-codegen')[0]


def t_benchmark1(solver):
    a = np.random.uniform(0.5, 2)
    b = np.random.uniform(0.5, 15)
    c = np.random.uniform(0.9, 3)
    _sol = solver.run([a, b, c])


def t_benchmark1p(solver):
    a = np.random.uniform(0.5, 2)
    b = np.random.uniform(0.5, 15)
    c = np.random.uniform(0.9, 5)
    _sol = solver.run([a, b, c])


def test_benchmark1(benchmark):
    benchmark.pedantic(t_benchmark1, kwargs={'solver': solver1}, iterations=50, rounds=2000)


def test_benchmark1p(benchmark):
    benchmark.pedantic(t_benchmark1p, kwargs={'solver': solver1p}, iterations=10, rounds=2000)
