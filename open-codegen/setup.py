from setuptools import setup

setup(name='opengen',
      version='0.0.3',
      description='Optimization Engine Code Generator',
      long_description=read('README.md'),
      long_description_content_type='text/markdown',
      author=['Pantelis Sopasakis', 'Emil Fresk'],
      author_email='p.sopasakis@gmail.com',
      license='MIT License',
      packages=['opengen', 'opengen.builder', 'opengen.config', 'opengen.functions'],
      include_package_data=True,
      install_requires=[
          'jinja2', 'casadi', 'numpy'
      ],
      classifiers=[
            'Development Status :: 2 - Pre-Alpha',
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
      keywords = ['optimization', 'nonconvex', 'embedded'],
      url=(
            'https://github.com/alphaville/optimization-engine'
      ))
