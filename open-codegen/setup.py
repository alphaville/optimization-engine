from setuptools import setup

setup(name='opengen',
      version='0.0.1',
      description='Optimization Engine Code Generator',
      author=['Pantelis Sopasakis', 'Emil Fresk'],
      author_email='p.sopasakis@gmail.com',
      license='MIT License',
      packages=['opengen', 'opengen.builder', 'opengen.config', 'opengen.functions'],
      package_data={'templates':['*'], 'icasadi':['*']},
      install_requires=[
            'jinja2', 'casadi', 'numpy'
      ],
      zip_safe=False)
