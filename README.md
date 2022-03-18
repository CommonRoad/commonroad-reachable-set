## CommonRoad Reachable Set

### System Requirements
The software is written in Python 3.7 and C++17, and has been tested on Ubuntu 18.04.


### Dependencies
C++ Libraries:
* Boost
* OpenMP
* Eigen3
* yaml-cpp

The following header-only libraries is needed for the Python wrapper (which can be found under external/):
* pybind11

The following library is required for running the tests (optional):
* doctest

Furthermore, the [CommonRoad Drivability Checker](https://commonroad.in.tum.de/drivability-checker) and [CommonRoad Route Planner](https://gitlab.lrz.de/tum-cps/commonroad-route-planner) libraries are required.
The following C++ third-party dependencies are included with the Drivability Checker and built when installing the Drivability Checker library:
* FCL
* CCD
* s11n_library


### Installation
* create a `build` folder and `cd build`
* build the project using (replace the flags accordingly):

```bash
cmake -DCRDC_DIR="/path/to/drivability-checker-root" -DPYTHON_VER="X.X" -DCMAKE_BUILD_TYPE=Release..

make
```

* optionally: to run the tests, include the flag `-DADD_TESTS=ON`


### Running the Code
After successfully compiling the C++ code, run `commonroad_reachset/compute_reachable_set.py` The outputs will be stored in the `output/` folder.

### Documentation

run the following command in the root directory:

```bash
doxygen ./doc/Doxyfile
```

Then open `./doc/html/index.html`.

