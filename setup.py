# read the README file
from os import path

from commonroad_reach.__version__ import __version__
from setuptools import setup, find_packages

this_directory = path.abspath(path.dirname(__file__))

# from distutils.core import setup
from Cython.Build import cythonize

setup(name='commonroad_reach', version=__version__,
      description='',
      keywords='autonomous automated vehicles driving motion planning', url='https://commonroad.in.tum.de/',
      project_urls={},
      author='Cyber-Physical Systems Group, Technical University of Munich', author_email='commonroad@lists.lrz.de',
      license="BSD", packages=find_packages(exclude=['doc', 'tests', 'tutorials']),
      install_requires=[],
      extras_require={},
      classifiers=["Programming Language :: Python :: 3.8", "License :: OSI Approved :: BSD License",
                   "Operating System :: POSIX :: Linux",],
      data_files=[],
      include_package_data=True,
      ext_modules=cythonize("commonroad_reach/utility/util_py_grid_online_reach.pyx",
                            build_dir="build-cython")
      )
