from os import path

from commonroad_reach.__version__ import __version__
from setuptools import setup, find_packages

this_directory = path.abspath(path.dirname(__file__))

from Cython.Build import cythonize

# TODO: does not build/install C++ code
setup(name='commonroad-reach', version=__version__,
      description='',
      keywords='autonomous automated vehicles driving motion planning',
      url='https://commonroad.in.tum.de/',
      project_urls={},
      author='Cyber-Physical Systems Group, Technical University of Munich',
      author_email='commonroad@lists.lrz.de',
      license="BSD",
      packages=find_packages(exclude=['tests']),
      install_requires=["commonroad-io>=2021.3",
                        "commonroad-route-planner>=2022.1",
                        "omegaconf>=2.1.1",
                        "setuptools>=50.3.2",
                        "numpy>=1.19.2",
                        "shapely>=1.7.0",
                        "enum34>=1.1.10",
                        "imageio>=2.9.0",
                        "seaborn>=0.10.0",
                        "matplotlib>=3.3.3",
                        "scipy>=1.4.1",
                        "networkx>=2.5",
                        "opencv-python>=4.5"],
      extras_require={"tests": ["pytest>=3.8.0"]},
      classifiers=["Programming Language :: Python :: 3.8",
                   "License :: OSI Approved :: BSD License",
                   "Operating System :: POSIX :: Linux",],
      data_files=[],
      include_package_data=True,
      ext_modules=cythonize("commonroad_reach/utility/util_py_grid_online_reach.pyx",
                            build_dir="build-cython")
      )
