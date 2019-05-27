from setuptools import setup


setup(name='opengen',
      version='0.0.3',
      description='Optimization Engine Code Generator',
      author=['Pantelis Sopasakis', 'Emil Fresk'],
      author_email='p.sopasakis@gmail.com',
      license='MIT License',
      packages=['opengen', 'opengen.builder', 'opengen.config', 'opengen.functions'],
      include_package_data = True,
      install_requires=[
            'jinja2', 'casadi', 'numpy'
      ])
