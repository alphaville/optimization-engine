# Testing and benchmarking

## First things first

Firstly, you need to create a virtual environment, activate it, and install opengen using 
```
pip install .
```
from within `open-codegen`.


## Benchmarking

To run the benchmarks, you need to install:
```
pip install pytest-benchmark[histogram]
```

Then, firstly, create some necessary optimizers to be benchmarked

```
python prepare_benchmarks.py
```

Run this from within `open-codegen/opengen`. 
This will create a number of solvers to be benchmarked; 
these will be stored in `open-codegen/opengen/.python_test_build/benchmarkable`.

Then benchmark them with 
```
py.test test/benchmark_open.py --benchmark-warmup-iterations=20
```

To generate a fancy histogram (box plots) run
```
py.test test/benchmark_open.py --benchmark-warmup-iterations=20 --benchmark-histogram=out
# to convert the SVG file to PNG:
qlmanage -t -s 1000 -o . out.svg
```

The generated benchmark looks like this:

![benchmark](https://user-images.githubusercontent.com/125415/192870362-672aa90d-e589-422e-98b9-3c24a691082a.png)

## Testing

Run 
```
python -W ignore test/test_constraints.py -v
python -W ignore test/test.py -v
```