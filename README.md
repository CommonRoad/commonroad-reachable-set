## CommonRoad Reachable Set

### System Requirements
The software is written in C++ and Python 3.7 and tested on Ubuntu 18.04.


### Dependencies
C++ Libraries:
* Boost
* OpenMP
* Eigen3
* FCL
* CCD
* s11n_library
* yaml-cpp

The following header-only libraries are needed for the Python wrapper (can be found under external/):
* pybind11

Furthermore, the *CommonRoad-Drivability-Checker library* and the *CommonRoad-Curvilinear-Coordinatesystem library* are necessary.

### Running the Code
After successfully compiling the C++ code, run `commonroad_reachset/continuous_reachable_set.py` and `commonroad_reachset/semantic_reachable_set.py`. The outputs will be stored in the `evaluation/` folder.

### Documentation

run the following command in the root directory:

```bash
doxygen ./doc/Doxyfile
```

Then open `./doc/html/index.html`.

