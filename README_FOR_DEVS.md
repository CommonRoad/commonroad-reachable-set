## Setting up a Local Development Environment

### Working with the Python Code

1. Follow the instructions for building the code in the [readme](./README.md).
2. If you are using Anaconda, make sure to select the correct Python environment in your IDE.

### Working with the C++ Code

While the [setup script](./setup.py) will build the C++ library automatically, for development purposes it is more convenient to build the code directly via CMake.
In the following we assume that your Anaconda environment for CommonRoad-Reach is named `commonroad`.

To build the code from the command line run (using Python version X.Y.Z):
```bash
$ conda activate commonroad
(commonroad)$ mkdir build
(commonroad)$ cd build
(commonroad)$ cmake -DCRDC_DIR="/path/to/drivability-checker-root" -DPYTHON_VER="XY" ..
(commonroad)$ make
```

If you want to build the code from your IDE, extra steps are necessary to ensure that the build uses the Python version from your Anaconda environment.
In addition to the CMake flags indicated above, you also need to pass
```
-DPYTHON_INCLUDE_DIR=/path/to/anaconda3/envs/commonroad/include/pythonX.Y.m
-DPYTHON_EXECUTABLE=/path/to/anaconda3/envs/commonroad/bin/python
```

To set this up in CLion, go to `Project settings > Build, Execution, Deployment > CMake` and add the four flags to `CMake options`.

#### Running the C++ example

After building the code, you can run the [C++ example](./cpp/src/example.cpp) from within your Anaconda environment.
```bash
(commonroad)$ ./build/cpp/example
```

When running the example from your IDE, you need to point the `PYTHONHOME` environment variable to your Anaconda environment to make sure that the example is executed with the correct Python version.

In CLion you need to edit the run configuration for the example and put `PYTHONHOME=/path/to/anaconda3/envs/commonroad/` in the field `Environment variables`. 
