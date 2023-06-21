import os
import sys
import subprocess
import warnings
import platform
import pathlib
import glob

from commonroad_reach.__version__ import __version__
from setuptools.command.build_ext import build_ext
from setuptools import setup, find_packages, Extension
from sysconfig import get_paths

this_directory = os.path.abspath(os.path.dirname(__file__))


class CMakeExtension(Extension):
    name: str  # IDE somehow doesn't detect name without this line

    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        self.validate_cmake()
        super().run()

    def build_extension(self, ext):
        if isinstance(ext, CMakeExtension):
            self.build_cmake_extension(ext)
        else:
            super().build_extension(ext)

    def validate_cmake(self):
        cmake_extensions = [x for x in self.extensions if isinstance(x, CMakeExtension)]
        if len(cmake_extensions) > 0:
            try:
                _ = subprocess.check_output(['cmake', '--version'])
            except OSError:
                raise RuntimeError("CMake must be installed to build the following extensions: " +
                                   ", ".join(e.name for e in self.extensions))

    def build_cmake_extension(self, ext: CMakeExtension):
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

        # get path to the drivability checker root
        if 'CRDC_DIR' in os.environ:
            cmake_args += ['-DCRDC_DIR=' + os.environ['CRDC_DIR']]
        else:
            warnings.warn('\t\t\t Please specify the path to the drivability checker root folder!')
            return None

        # pass version of current python binary
        python_ver = platform.python_version()[0] + platform.python_version()[2]
        cmake_args += ['-DPYTHON_VER=' + python_ver]

        # add tests
        if 'ADD_TESTS' in os.environ:
            cmake_args += ['DADD_TESTS=' + os.environ['ADD_TESTS']]
        else:
            cmake_args += ['-DADD_TESTS=OFF']

        # CMake build type (debug / release)
        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]
        cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]

        # number of Build Jobs
        if 'BUILD_JOBS' in os.environ:
            build_args += ['--'] + ['-j'] + [os.environ['BUILD_JOBS']]

        print(cmake_args)
        print(build_args)

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        # build
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=build_dir)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=build_dir)

        # copy files
        install_dir = self.get_ext_fullpath(ext.name)
        extension_install_dir = pathlib.Path(install_dir).parent.joinpath(ext.name).resolve()
        for file in glob.glob((f'{pathlib.Path(build_dir).parent.resolve()}/' + 'pycrreach.*.so')):
            self.copy_file(file, extension_install_dir)
            self.copy_file(file, os.path.join(os.getcwd(), 'commonroad_reach'))


setup(name='commonroad-reach', version=__version__,
      description='CommonRoad Reach: A Toolbox for Computing Reachable Sets of Automated Vehicles',
      keywords='autonomous automated vehicles driving motion planning',
      url='https://commonroad.in.tum.de/tools/commonroad-reach',
      project_urls={
          'Documentation': 'https://commonroad.in.tum.de/docs/commonroad-reach/',
          'Forum': 'https://commonroad.in.tum.de/forum/c/comonroad-reach/19',
          'Source': 'https://gitlab.lrz.de/tum-cps/commonroad-reach'
      },
      author='Cyber-Physical Systems Group, Technical University of Munich',
      author_email='commonroad@lists.lrz.de',
      license="BSD",
      packages=find_packages(exclude=['commonroad_reach.tests']),

      ext_modules=[
          CMakeExtension("commonroad_reach"),
      ],
      cmdclass={"build_ext": CMakeBuild},
      python_requires='>=3.7',
      install_requires=["commonroad-io>=2023.1",
                        "commonroad-route-planner>=2022.3",
                        "commonroad-drivability-checker>=2022.2",
                        "cython>=0.29.28",
                        "omegaconf>=2.1.1",
                        "setuptools>=62.1.0",
                        "numpy>=1.19.2",
                        "shapely>=2.0.0",
                        "enum34>=1.1.10",
                        "imageio>=2.9.0",
                        "seaborn>=0.10.0",
                        "matplotlib>=3.3.3",
                        "scipy>=1.4.1",
                        "networkx>=2.5",
                        "opencv-python>=4.5"],
      extras_require={"tests": ["pytest>=3.8.0"]},
      classifiers=["Programming Language :: C++",
                   "Programming Language :: Python :: 3.7",
                   "Programming Language :: Python :: 3.8",
                   "Programming Language :: Python :: 3.9",
                   "License :: OSI Approved :: BSD License",
                   "Operating System :: POSIX :: Linux",],
      data_files=[],
      include_package_data=True,
      )
