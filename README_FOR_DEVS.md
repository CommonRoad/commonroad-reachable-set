## Building from Source

> **Note:** Currently there appears to be a bug with boost geometry and newer versions of GCC (this seems to start with version 11.4).
> A workaround until this is fixed is to use an older version of GCC (we suggest GCC 10).
> To do so, indicate the path to the older version of GCC in the `CXX` environment variable before building the code (e.g. `export CXX=/usr/bin/g++-10`).

### Third-Party Dependencies

The following third-party dependencies of the C++ code are only required for building the project from source!
While most of these dependencies are added automatically during the build process, you can install them manually via your package manager to speed up the build process.

**Manual installation required:**
- [OpenMP](https://www.openmp.org/)

**Manual installation recommended to speed up the build:**
- [Boost.Geometry](https://www.boost.org/doc/libs/1_79_0/libs/geometry/doc/html/index.html)

**Manual installation optional:**
- [CommonRoad Drivability Checker](https://commonroad.in.tum.de/tools/drivability-checker) (version >= 2023.1)
- [Eigen3](https://eigen.tuxfamily.org/)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [pybind11](https://github.com/pybind/pybind11)

**Optional dependencies:**
- [Doctest](https://github.com/doctest/doctest) (optional: for building unit tests)
- [Doxygen](https://doxygen.nl/) (optional: for building documentation)

The additional Python dependencies are listed in `pyproject.toml`.


### Building the Code

1. Install C++ dependencies:
  ```bash
  sudo apt-get update
  sudo apt-get install libomp-dev libboost-all-dev libeigen3-dev libyaml-cpp-dev pybind11-dev doctest-dev doxygen
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

1. Install the C++ dependencies as described [above](#third-party-dependencies).

2. Install the Python build dependencies (required to make `--no-build-isolation` work in the next step):
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

## Debugging the C++ Code

1. Install the package in editable mode using a Debug build:
```bash
pip install -v --no-build-isolation --config-settings=editable.rebuild=true --config-settings=cmake.build-type="Debug" -e .
```

2. Launch the Python interpreter under a C++ debugger, for example with GDB:
```bash
gdb -ex r --args python compute_reachable_set.py
```

You can also use your favorite IDE to debug the C++ code.

### Debugging with CLion

To set up a debugging configuration with CLion, follow the steps described under [option 2 here](https://www.jetbrains.com/help/clion/debugging-python-extensions.html#debug-custom-py).
Make sure to use the Python and pip executables from your Anaconda environment.

When setting up the external build tool in CLion, we recommend to choose a different build directory to avoid interference with your manual builds.
You also have to make sure that CMake uses the correct compiler version (see the note at the very top of this document).
Below, you find the pip arguments of an example configuration:
```
install
-v
--no-build-isolation
--config-settings=editable.rebuild=true
--config-settings=cmake.build-type="Debug"
--config-settings=cmake.define.CMAKE_CXX_COMPILER=/usr/bin/g++-10
--config-settings=build-dir=build/CLion
-e
.
```

> **Note:** Do not disable the automatic rebuilds. Otherwise, CLion appears to not recognize the breakpoints you set.
> It also appears that breakpoints are not recognized if you start debugging immeadiately after changing the code.
> In this case, restarting the debugging session should help.

Alternatively, you can omit the build step in the CLion configuration and just relay on the automatic rebuilds of your manual debug installation.
With this, the breakpoints seem to work more reliably.
To do so, edit your run configuration and remove "Build" from the "Before launch" section.

If all else fails, uninstalling and reinstalling the package also seems to fix the breakpoint recognition.
