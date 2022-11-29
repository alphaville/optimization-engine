#!/usr/bin/env python

from setuptools import setup, find_packages
import io
import os

here = os.path.abspath(os.path.dirname(__file__))

NAME = 'opengen'

# Import version from file
version_file = open(os.path.join(here, 'VERSION'))
VERSION = version_file.read().strip()

DESCRIPTION = 'Optimization Engine Code Generator'


# Import the README and use it as the long-description.
# Note: this will only work if 'README.md' is present in your MANIFEST.in file!
try:
    with io.open(os.path.join(here, 'README.md'), encoding='utf-8') as f:
        long_description = '\n' + f.read()
except FileNotFoundError:
    long_description = DESCRIPTION

setup(name=NAME,
      version=VERSION,
      description=DESCRIPTION,
      long_description=long_description,
      long_description_content_type='text/markdown',
      author=['Pantelis Sopasakis', 'Emil Fresk'],
      author_email='p.sopasakis@gmail.com',
      license='MIT License',
      packages=find_packages(
          exclude=["icasadi", "templates"]),
      include_package_data=True,
      install_requires=[
          'jinja2', 'casadi', 'pyyaml', 'retry', 'numpy'
      ],
      classifiers=[
          'Development Status :: 4 - Beta',
          'License :: OSI Approved :: MIT License',
          'License :: OSI Approved :: Apache Software License',
          'Programming Language :: Python',
          'Programming Language :: Rust',
          'Intended Audience :: Science/Research',
          'Topic :: Software Development :: Libraries',
          'Topic :: Scientific/Engineering',
          'Topic :: Scientific/Engineering :: Mathematics',
          'Topic :: Software Development :: Code Generators',
          'Topic :: Software Development :: Embedded Systems'
      ],
      keywords=['optimization', 'nonconvex', 'embedded'],
      url=(
          'https://github.com/alphaville/optimization-engine'
      ),
      zip_safe=False)
