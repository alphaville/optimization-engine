import time
import casadi.casadi as cs
import opengen as og
import os
import sys
import numpy as np
from importlib import import_module

ITERS = 10
ROUNDS = 2000

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

# TODO automate this...
for name in os.listdir(TEST_DIR):
    sys.path.insert(1, os.path.join(TEST_DIR, name))
    exec(f"{name} = import_module(\"{name}\")")
    exec(f"solver_{name} = {name}.solver()")


def get_open_local_absolute_path():
    cwd = os.getcwd()
    return cwd.split('open-codegen')[0]


def t_benchmark1(solver):
    a = np.random.uniform(0.5, 2)
    b = np.random.uniform(0.5, 15)
    c = np.random.uniform(0.9, 3)
    _sol = solver.run([a, b, c])


def t_benchmark2(solver):
    x0 = np.random.uniform(-2, -1)
    y0 = np.random.uniform(-1.5, -1.5)
    th0 = np.random.uniform(-0.5, 0.5)
    _sol = solver.run([x0, y0, th0])


def test_benchmark1(benchmark):
    benchmark.pedantic(t_benchmark1, kwargs={'solver': solver_benchmark1},
                       iterations=ITERS, rounds=ROUNDS)


def test_benchmark1p(benchmark):
    benchmark.pedantic(t_benchmark1, kwargs={'solver': solver_benchmark1p},
                       iterations=ITERS, rounds=ROUNDS)


def test_benchmark2(benchmark):
    benchmark.pedantic(t_benchmark2, kwargs={'solver': solver_benchmark2},
                       iterations=ITERS, rounds=ROUNDS)


def test_benchmark2p(benchmark):
    benchmark.pedantic(t_benchmark2, kwargs={'solver': solver_benchmark2p},
                       iterations=ITERS, rounds=ROUNDS)