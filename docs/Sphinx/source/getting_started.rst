###############
Getting Started
###############

.. _Anaconda: http://www.anaconda.com/download/#download

*******************
System Requirements
*******************
The software is written in Python 3.7 and C++17, and was tested on Ubuntu 18.04. It should be compatible with later linux distributions.

**********************************
Third Party Libraries and Packages
**********************************
The C++ code depends on the following third party libraries:

- `CommonRoad Drivability Checker <https://commonroad.in.tum.de/drivability-checker>`_
- `Boost.Geometry <https://www.boost.org/doc/libs/1_79_0/libs/geometry/doc/html/index.html>`_
- `OpenMP <https://www.openmp.org/>`_
- `yaml-cpp <https://github.com/jbeder/yaml-cpp>`_
- `pybind11 <https://github.com/pybind/pybind11>`_
- `Doctest <https://github.com/doctest/doctest>`_ (optional: for building unit tests)
- `Doxygen <https://doxygen.nl/>`_ (optional: for documentation)

Required Python dependencies are listed in :guilabel:`requirement.txt`.

*****************
Building the Code
*****************
We strongly recommend using Anaconda_ to manage Python virtual environments.

#. Install Python dependencies:

    .. code-block:: console

        $ pip install -r requirements.txt

#. Check the following minimum required versions and update them if necessary:

    - ``GCC and G++``: version 9.0 or above
    - ``CMake``: version 3.15 or above

#. Install `CommonRoad Drivability Checker <https://commonroad.in.tum.de/drivability-checker>`_. Please refer to its `documentation <https://commonroad.in.tum.de/docs/commonroad-drivability-checker/sphinx/installation.html>`_ for installation.

#. Install yaml-cpp and Doctest:

    .. code-block:: console

        $ sudo apt update
        $ sudo apt install libyaml-cpp-dev
        $ sudo apt install doctest-dev

#. Install/upgrade OpenMP:

    .. code-block:: console

        $ sudo apt-get install libomp-dev
        $ sudo apt upgrade libomp-dev

#. Build the package and install it to your conda environment via pip command:

    .. code-block:: console

        $ CRDC_DIR="/path/to/commonroad-drivability-checker/" pip install -v .

    This will build the python binding (pycrreach) required for collision checks and other C++-boosted computations.

.. note::
    - Replace ``"/path/to/commonroad-drivability-checker/"`` with your local path to the Drivability Checker directory.

    - The ``-v`` flag prints verbose information about the build progress.

    **Optional**:
    
    - To add unit tests,  set variable ``ADD_TESTS=ON`` before the pip command.

    - To build the code in Debug mode, set ``debug=1`` in the setup configuration file (:guilabel:`setup.cfg`).

****************
Running the Code
****************
Tutorial Jupyter notebooks are available at ``./tutorials/``. Exemplary scripts for computing reacahble sets and extracint driving corridors are also provided:

    - To compute reachable sets, run ``commonroad_reachset/compute_reachable_set.py``.
    
    - To extract driving corridors, run ``commonroad_reachset/extract_driving_corridors.py``.

The outputs will be stored in the ``./output/`` folder. Default and scenario-specific configurations are stored in the ``./configurations/`` folder.

*********************
Doxygen Documentation
*********************
Run the following command in the root directory to generate C++ documentation. 

.. code-block:: console

        $ doxygen ./docs/Doxyfile

Doxygen documentation can be launched by browsing ``./docs/Doxygen/html/index.html/``.
