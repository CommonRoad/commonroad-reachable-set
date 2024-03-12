## Building from Source

> **Note:** Currently there appears to be a bug with boost geometry and newer versions of GCC (this seems to start with version 11.4).
> A workaround until this is fixed is to use an older version of GCC (we suggest GCC 10).
> To do so, indicate the path to the older version of GCC in the `CXX` environment variable before building the code (e.g. `export CXX=/usr/bin/g++-10`).

### Third-Party Dependencies

The following third-party dependencies of the C++ code are only required for building the project from source!
While most of these dependencies are added automatically during the build process, you can install them manually via your package manager to speed up the build process.

**Manual installation required:**
* [OpenMP](https://www.openmp.org/)

**Manual installation recommended to speed up the build:**
* [Boost.Geometry](https://www.boost.org/doc/libs/1_79_0/libs/geometry/doc/html/index.html)

**Manual installation optional:**
* [CommonRoad Drivability Checker](https://commonroad.in.tum.de/tools/drivability-checker) (version >= 2023.1)
* [yaml-cpp](https://github.com/jbeder/yaml-cpp)
* [pybind11](https://github.com/pybind/pybind11)

**Optional dependencies:**
* [Doctest](https://github.com/doctest/doctest) (optional: for building unit tests)
* [Doxygen](https://doxygen.nl/) (optional: for building documentation)

The additional Python dependencies are listed in `pyproject.toml`.


### Building the Code

1. Install C++ dependencies:
  ```bash
  sudo apt-get update
  sudo apt-get install libboost-all-dev libyaml-cpp-dev libomp-dev doctest-dev doxygen
  ```

2. Build the package and install it to your conda environment via pip command.
  ```bash
  pip install -v .
  ```
  This will build the Python binding (pycrreach) required for collision checks and other C++-boosted computations.

> **Note**: The `-v` flag (verbose) prints information about the build progress

**Optional:**

- To build the code in Debug mode, add the flag `--config-settings=cmake.build-type="Debug"` to the `pip` command.
- See [here](https://scikit-build-core.readthedocs.io/en/latest/configuration.html#configuring-cmake-arguments-and-defines) for further information on configuring CMake arguments via our build system (`scikit-build-core`).

> **Note**: `scikit-build-core` uses `ninja` for building the C++ extension by default.
> Thus, the build is automatically parallelized using all available CPU cores.
> If you want to explicitly configure the number of build jobs, you can do so by passing the flag `--config-settings=cmake.define.CMAKE_BUILD_PARALLEL_LEVEL=$BUILD_JOBS` to the `pip` command, where `$BUILD_JOBS` is the number of parallel jobs to use.
> See [here](https://scikit-build-core.readthedocs.io/en/latest/faqs.html#multithreaded-builds) for further details.

> **Note**: Building the package in Debug mode (see above) significantly increases the computation time of the C++ backend. Please make sure you are building in Release mode (default setting) if you require fast computations.

## Editable Install (experimental)

1. Install the third-party C++ dependencies as described [above](#third-party-dependencies).

2. Install the Python build dependencies:
```bash
pip install -r requirements_build.txt
```

3. Build the package and install it in editable mode with automatic rebuilds.
```bash
pip install -v --no-build-isolation --config-settings=editable.rebuild=true -e .
```
Note that this is considered experimental by `scikit-build-core` and is subject to change.
For more information, please see the [documentation](https://scikit-build-core.readthedocs.io/en/latest/configuration.html#editable-installs) of `scikit-build-core`.
Flags:
- `-v` (verbose) prints information about the build progress
- `--no-build-isolation` disables build isolation, which means the build runs in your local environment
- `--config-settings=editable.rebuild=true` enables automatic rebuilds when the source code changes (see the caveats in the documentation of `scikit-build-core`)
- `-e` (editable) installs the package in editable mode

## Setting up a Local Development Environment

### Working with the Python Code

1. Follow the instructions for building the code in the [readme](./README.md).
2. If you are using Anaconda, make sure to select the correct Python environment in your IDE.

### Working with the C++ Code

While `scikit-build-core` will build the C++ library automatically when installing the Python package, for development purposes it might be more convenient to build the code directly via CMake.
In the following we assume that your Anaconda environment for CommonRoad-Reach is named `commonroad`.

To build the code from the command line run (using Python version X.Y.Z):
```bash
conda activate commonroad
mkdir build && cd build
cmake ..
cmake --build . -j $BUILD_JOBS
```

If you want to build the code from your IDE, extra steps are necessary to ensure that the build uses the Python version from your Anaconda environment.
You need to pass the following flags to CMake:
```
-DPYTHON_INCLUDE_DIR=/path/to/anaconda3/envs/commonroad/include/pythonX.Y
-DPYTHON_EXECUTABLE=/path/to/anaconda3/envs/commonroad/bin/python
```

To set this up in CLion, go to `Project settings > Build, Execution, Deployment > CMake` and add the flags to `CMake options`.

#### Running the C++ example

After building the code, you can run the [C++ example](./cpp/src/example.cpp) from within your Anaconda environment.
```bash
conda activate commonroad
./build/cpp/example
```

When running the example from your IDE, you need to point the `PYTHONHOME` environment variable to your Anaconda environment to make sure that the example is executed with the correct Python version.

In CLion you need to edit the run configuration for the example and put `PYTHONHOME=/path/to/anaconda3/envs/commonroad/` in the field `Environment variables`. 
