# CommonRoad-Reach: A Toolbox for Reachability Analysis of Automated Vehicles

Reachability analysis has gained increasing popularity in motion planning and safeguarding of automated vehicles (AVs). 
While existing tools for reachability analysis mainly focus on general-purpose algorithms for formal verification 
of dynamical systems, a toolbox tailored to AV-specific applications is not yet available. 
The CommonRoad-Reach toolbox

- integrates different methods for computing reachable sets using polytopic set propagation and graph-based propagation;
- provides Python and C++ implementations of the algorithms, thus offering convenient prototyping and real-time computation for the users; and
- extracts driving corridors which can be used as planning constraints for motion planners.


## System Requirements

The software is written in Python 3.10 and C++17, and was tested on Ubuntu 18.04, 20.04 and 22.04.
It should be compatible with later versions.
For building the code, the following minimum versions are required:
  * **GCC and G++**: version 10 or above
  * **CMake**: version 3.20 or above.
  * **Pip**: version 21.3 or above

We further recommend using [Anaconda](https://www.anaconda.com/) to manage your virtual python environment.


## Installation options

We provide two installation options for CommonRoad-Reach: Installation as a Python package or building from source.

1. **Python Package**: Install the python package via `pip` in your Conda environment:
    ```bash
    pip install commonroad-reach
    ```

2. **Build from source**: To build the project from source and install it in your Conda environment, please refer to the [README_FOR_DEVS](./README_FOR_DEVS.md).
This option is only recommended for advanced users and those who are looking to contribute to the development of CommonRoad-Reach.

## Getting Started

Run the exemplary scripts to compute reachable sets and extract driving corridors.

* To compute reachable sets, run `compute_reachable_set.py`.

* To extract driving corridors, run `extract_driving_corridors.py`.

The outputs will be stored in the `./output/` folder. Default and scenario-specific configurations are stored in the `./configurations/` folder.


## Documentation

The documentation of our toolbox is available on our website: https://cps.pages.gitlab.lrz.de/commonroad/commonroad-reachable-set/.

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
