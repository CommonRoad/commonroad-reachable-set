# CommonRoad-Reach: A Toolbox for Reachability Analysis of Automated Vehicles

Reachability analysis has gained increasing popularity in motion planning and safeguarding of automated vehicles (AVs). 
While existing tools for reachability analysis mainly focus on general-purpose algorithms for formal verification 
of dynamical systems, a toolbox tailored to AV-specific applications is not yet available. 
The CommonRoad-Reach toolbox

- integrates different methods for computing reachable sets using polytopic set propagation and graph-based propagation;
- provides Python and C++ implementations of the algorithms, thus offering convenient prototyping and real-time computation for the users; and
- extracts driving corridors which can be used as planning constraints for motion planners.


## System Requirements

The software is written in Python 3.7 and C++17, and was tested on Ubuntu 18.04 and Ubuntu 22.04.
It should be compatible with later versions.
For building the code, the following minimum versions are required:
  * **GCC and G++**: version 9 or above
  * **CMake**: version 3.15 or above.
  * **Pip**: version 21.3 or above

We further recommend using [Anaconda](https://www.anaconda.com/) to manage your virtual python environment.


## Installation options

We provide two installation options for CommonRoad-Reach: Installation as a Python package or building from source.

1. **Python Package**: Install the python package via `pip` in your Conda environment:
    ```bash
    pip install commonroad-reach
    ```

2. **Build from source**: To build the project from source and install it in your Conda environment, please refer to the
descriptions below.


## Building from Source

> **Note:** Currently there appears to be a bug with boost geometry and newer versions of GCC (this seems to start with version 11.4).
> A workaround until this is fixed is to use an older version of GCC (we suggest GCC 10).
> To do so, indicate the path to the older version of GCC in the `CXX` environment variable before building the code (e.g. `export CXX=/usr/bin/g++-10`).

### Third Party Dependencies
The following third-party dependencies of the C++ code are only required for building the project from source!

**Essential dependencies**:
* [CommonRoad Drivability Checker](https://commonroad.in.tum.de/tools/drivability-checker) (version >= 2023.1)
* [Boost.Geometry](https://www.boost.org/doc/libs/1_79_0/libs/geometry/doc/html/index.html)
* [OpenMP](https://www.openmp.org/)
* [yaml-cpp](https://github.com/jbeder/yaml-cpp)
* [pybind11](https://github.com/pybind/pybind11)

**Optional dependencies (testing and documentation)**:
* [Doctest](https://github.com/doctest/doctest) (optional: for building unit tests)
* [Doxygen](https://doxygen.nl/) (optional: for building documentation)

The additional Python dependencies are listed in `requirements.txt`.


### Building the Code

* Install Boost, yaml-cppy and Doctest:
  ```bash
  sudo apt-get update
  sudo apt-get install libboost-all-dev libyaml-cpp-dev doctest-dev
  ```

* Install/upgrade OpenMP:
  ```bash
  sudo apt-get install libomp-dev
  sudo apt-get upgrade libomp-dev
  ```

* Build the package and install it to your conda environment via pip command:
  ```bash
  pip install -v .
  ```
  This will build the Python binding (pycrreach) required for collision checks and other C++-boosted computations.

> **Note**: The `-v` flag (verbose) prints information about the build progress

**Optional:**

- To add unit tests, add the flag `--config-settings=cmake.define.ADD_TESTS=ON` to the `pip` command.
- To build the code in Debug mode, add the flag `--config-settings=cmake.build-type="Debug"` to the `pip` command.
- See [here](https://scikit-build-core.readthedocs.io/en/latest/configuration.html#configuring-cmake-arguments-and-defines) for further information on configuring CMake arguments via our build system (`scikit-build-core`).

With both optional steps, the `pip` command looks as follows:

```bash
pip install -v . --config-settings=cmake.build-type="Debug" --config-settings=cmake.define.ADD_TESTS=ON
```

> **Note**: `scikit-build-core` uses `ninja` for building the C++ extension by default.
> Thus, the build is automatically parallelized using all available CPU cores.
> If you want to explicitly configure the number of build jobs, you can do so by passing the flag `--config-settings=cmake.define.CMAKE_BUILD_PARALLEL_LEVEL=$BUILD_JOBS` to the `pip` command, where `$BUILD_JOBS` is the number of parallel jobs to use.
> See [here](https://scikit-build-core.readthedocs.io/en/latest/faqs.html#multithreaded-builds) for further details.


## Getting Started

Run the exemplary scripts to compute reachable sets and extract driving corridors.

* To compute reachable sets, run `compute_reachable_set.py`.

* To extract driving corridors, run `extract_driving_corridors.py`.

The outputs will be stored in the `./output/` folder. Default and scenario-specific configurations are stored in the `./configurations/` folder.


## Documentation

The documentation of our toolbox is available on our website: https://cps.pages.gitlab.lrz.de/commonroad-reachable-set/.

In order to generate the documentation via Sphinx locally, run the following commands in the root directory:

```bash
pip install -r ./docs/requirements_doc.txt
cd docs/Sphinx
make html
```

The documentation can then be launched by browsing ``./docs/Sphinx/build/html/index.html/``.


## Citation
If you use our toolbox for your research, please cite our [paper](https://mediatum.ub.tum.de/doc/1684928/1684928.pdf):

```text
@InProceedings{iraniliu2022commonroad,
      title     = {{CommonRoad-Reach}: {A} toolbox for reachability analysis of automated vehicles},
      author    = {Irani Liu, Edmond and W\"ursching, Gerald and Klischat, Moritz and Althoff, Matthias},
      booktitle = {Proc. of the IEEE Int. Conf. Intell. Transp. Syst.},
      pages     = {2313--2320},
      year      = {2022},
   }
```

## Development

If you want to set up a local development environment, please refer to the [README_FOR_DEVS](./README_FOR_DEVS.md).
