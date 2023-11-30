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
