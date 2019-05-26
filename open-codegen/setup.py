from setuptools import setup


setup(name='opengen',
      version='0.0.1',
      description='Optimization Engine Code Generator',
      author=['Pantelis Sopasakis', 'Emil Fresk'],
      author_email='p.sopasakis@gmail.com',
      license='MIT License',
      packages=['opengen', 'opengen.builder', 'opengen.config', 'opengen.functions'],
      data_files=[('templates',
                   ['templates/icasadi_config.h.template',
                    'templates/optimizer_cargo.toml.template',
                    'templates/optimizer.rs.template']),
                  ('icasadi',
                   ['icasadi/build.rs',
                    'icasadi/Cargo.toml']),
                  ('icasadi/extern',
                   ['icasadi/extern/icasadi.c',
                    'icasadi/extern/icasadi.h',
                    'icasadi/extern/icasadi_config.h'])],
      install_requires=[
            'jinja2', 'casadi', 'numpy'
      ],
      zip_safe=False)
