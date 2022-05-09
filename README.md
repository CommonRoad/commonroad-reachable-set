## CommonRoad Reach: Reachability Analysis for Automated Vehicles

In recent years, reachability analysis has gained increasing popularity in motion planning and safeguarding of automated vehicles (AVs). While existing tools for reachability analysis mainly focus on general-purpose algorithms for formal verification of dynamical systems, a toolbox tailored to AV-specific applications is not yet available. The CommonRoad Reach toolbox
* integrates different methods for computing reachable sets using polytopic set propagation and graph-based propagation;
* provides Python and C++ implementations of the algorithms, thus offering convenient prototyping and real-time computation for the users;
* extracts collision-free driving corridors which can be used as planning spaces for motion planners; and
* is integrated within the CommonRoad benchmark suite.

### Dependencies

The software is written in Python 3.7 and C++17, and was tested on Ubuntu 18.04. It should be compatible with later versions.

The C++ code depends on the following libraries:

* CommonRoad Drivability Checker

* Boost.Geometry
* Eigen3
* CCD
* FCL
* s11n
* OpenMP
* Yaml-cpp
* pybind11
* Doctest (optional: for building unit tests)

The Python dependencies are listed in `requirements.txt`.

### Building the code

* `Optional:` We recommend using [Anaconda](https://www.anaconda.com/) to manage your virtual python environment.

* Install Python dependencies:

  ```bash
  pip install -r requirements.txt
  ```

* Check the following minimum required versions and update if necessary:
  * **GCC and G++**: version 9 or above
  * **CMake**: version 3.15 or above.
  * **Pip**: version 21.3 or above

* Install [CommonRoad Drivability Checker](https://commonroad.in.tum.de/drivability-checker). Please refer to its [documentation](https://commonroad.in.tum.de/docs/commonroad-drivability-checker/sphinx/installation.html) for installation.

* Install Yaml and Doctest:
  ```bash
  $ sudo apt update
  $ sudo apt install libyaml-cpp-dev
  $ sudo apt install doctest-dev
  ```

* Install/upgrade OpenMP:

  ```bash
  $ sudo apt-get install libomp-dev
  $ sudo apt upgrade libomp-dev
  ```

* Build the package and install it in your conda environment via pip:

  ```bash
  $ CRDC_DIR="/path/to/commonroad-drivability-checker" pip install -v .
  ```
  
**Note**: 

  * Replace `"/path/to/drivability-checker-folder"` with the path to the Drivability Checker folder on your machine.
  * the `-v` flag (verbose) prints information about the build progress

**Optional:**
  * to add unit tests you can add the environment variable `ADD_TESTS=ON` before the `pip install` command
  * to build the code in CMake Debug Mode: set `debug=1` in the setup configuration file (`setup.cfg`)

### Running the code

* Run `commonroad_reachset/compute_reachable_set.py` , the outputs will be stored in the `./output/` folder.
* Run `commonroad_reachset/compute_driving_corridors.py`, the outputs will be stored in the `./output/` folder

### Documentation

Run the following command in the root directory, the documentation is accessible at `./doc/html/index.html`.

```bash
doxygen ./doc/Doxyfile
```

### Citation

to be added.
