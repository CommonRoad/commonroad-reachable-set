## CommonRoad-Reach: A Toolbox for Reachability Analysis of Automated Vehicles

Reachability analysis has gained increasing popularity in motion planning and safeguarding of automated vehicles (AVs). While existing tools for reachability analysis mainly focus on general-purpose algorithms for formal verification of dynamical systems, a toolbox tailored to AV-specific applications is not yet available. The CommonRoad-Reach toolbox

- integrates different methods for computing reachable sets using polytopic set propagation and graph-based propagation;
- provides Python and C++ implementations of the algorithms, thus offering convenient prototyping and real-time computation for the users; and
- extracts driving corridors which can be used as planning constraints for motion planners.

### System Requirements

The software is written in Python 3.7 and C++17, and was tested on Ubuntu 18.04. It should be compatible with later 
versions. For building the code, the following minimum versions are required:
  * **GCC and G++**: version 9 or above
  * **CMake**: version 3.15 or above.
  * **Pip**: version 21.3 or above


### Third Party Libraries and Packages

The C++ code depends on the following libraries: 

**Essential dependencies**:
* [CommonRoad Drivability Checker](https://commonroad.in.tum.de/drivability-checker)
* [Boost.Geometry](https://www.boost.org/doc/libs/1_79_0/libs/geometry/doc/html/index.html)
* [OpenMP](https://www.openmp.org/)
* [yaml-cpp](https://github.com/jbeder/yaml-cpp)
* [pybind11](https://github.com/pybind/pybind11)
  
**Optional dependencies**:
* [Doctest](https://github.com/doctest/doctest) (optional: for building unit tests)
* [Doxygen](https://doxygen.nl/) (optional: for building documentation)

The Python dependencies are listed in `requirements.txt`.

### Building the Code

* `Optional:` We recommend using [Anaconda](https://www.anaconda.com/) to manage your virtual python environment.

* Install Python dependencies:

  ```bash
  $ pip install -r requirements.txt
  ```

* Install [CommonRoad Drivability Checker](https://commonroad.in.tum.de/drivability-checker). Please refer to its [documentation](https://commonroad.in.tum.de/docs/commonroad-drivability-checker/sphinx/installation.html) for installation.

* Install yaml-cpp and Doctest:
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

* Build the package and install it to your conda environment via pip command:

  ```bash
  $ CRDC_DIR="/path/to/commonroad-drivability-checker/" pip install -v .
  ```
  This will build the python binding (pycrreach) required for collision checks and other C++-boosted computations.


**Note**: 

  * Replace `"/path/to/commonroad-drivability-checker/"` with the path to the Drivability Checker folder on your machine.
  * The `-v` flag (verbose) prints information about the build progress

**Optional:**

- To install the package in editable mode, add flag `-e` to the pip command above.
- To add unit tests,  set variable `ADD_TESTS=ON` before the pip command.
- To build the code in Debug mode, set `debug=1` in the setup configuration file (`setup.cfg`).

### Running the Code

Run the exemplary scripts to compute reachable sets and extract driving corridors.

* To compute reachable sets, run ``commonroad_reachset/compute_reachable_set.py``.

- To extract driving corridors, run ``commonroad_reachset/extract_driving_corridors.py``.

The outputs will be stored in the ``./output/`` folder. Default and scenario-specific configurations are stored in the ``./configurations/`` folder.

### Doxygen Documentation

Run the following command in the root directory to generate C++ documentation. 

```bash
doxygen ./docs/Doxyfile
```

Doxygen documentation can be launched by browsing ``./docs/Doxygen/html/index.html/``.

### Citation

```text
@InProceedings{iraniliu2022commonroad,
      author    = {Irani Liu, Edmond and W\"ursching, Gerald and Klischat, Moritz and Althoff, Matthias},
      booktitle = {Proc. of the IEEE Int. Conf. Intell. Transp. Syst.},
      title     = {{CommonRoad-Reach}: {A} toolbox for reachability analysis of automated vehicles},
      year      = {2022},
      pages     = {1--8}
   }
```

