import os
import sys
import subprocess

# from commonroad_reach.__version__ import __version__
from setuptools import setup, find_packages, Extension
from setuptools.command.build_ext import build_ext
from sysconfig import get_paths

this_directory = os.path.abspath(os.path.dirname(__file__))

from Cython.Build import cythonize


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            _ = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        build_dir = os.path.abspath(self.build_temp)

        default_python_include_dir = get_paths()['include']
        default_python_executable = sys.executable

        if 'PYTHON_INCLUDE_DIR' in os.environ:
            python_include_dir = os.environ['PYTHON_INCLUDE_DIR']
        else:
            python_include_dir = default_python_include_dir

        if 'PYTHON_EXECUTABLE' in os.environ:
            python_executable = os.environ['PYTHON_EXECUTABLE']
        else:
            python_executable = default_python_executable

        cmake_args = [
            "-DPYTHON_INCLUDE_DIR=" + python_include_dir,
            "-DPYTHON_EXECUTABLE=" + python_executable,
        ]

        # TODO: add CRDC_DIR and PYTHON_VER

        # add tests
        if 'ADD_TESTS' in os.environ:
            cmake_args += ['DADD_TESTS=' + os.environ['ADD_TESTS']]
        else:
            cmake_args += ['-DADD_TESTS=OFF']

        # CMake build type (debug / release)
        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]

        if 'BUILD_JOBS' in os.environ:
            build_args += ['--'] + ['-j'] + [os.environ['BUILD_JOBS']]

        print(cmake_args)
        print(build_args)

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=build_dir)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=build_dir)


# TODO: does not build/install C++ code
setup(name='commonroad-reach', version=2022.1, #__version__,
      description='CommonRoad-Reach: A Toolbox for Computing Reachable Sets of Automated Vehicles',
      keywords='autonomous automated vehicles driving motion planning',
      url='https://commonroad.in.tum.de/',
      project_urls={
          'Documentation': '',
          'Forum': '',
          'Source': ','
      },
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
      classifiers=["Programming Language :: C++",
                   "Programming Language :: Python :: 3.8",
                   "License :: OSI Approved :: BSD License",
                   "Operating System :: POSIX :: Linux",],
      data_files=[],
      include_package_data=True,
      ext_modules=[
          CMakeExtension("commonroad_reach"),
          cythonize("commonroad_reach/utility/util_py_grid_online_reach.pyx", build_dir="build-cython")
      ]
      )
