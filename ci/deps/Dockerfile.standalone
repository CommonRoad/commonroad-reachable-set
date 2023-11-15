FROM gitlab.lrz.de:5005/cps/rapid-prototyping-setup/ci-lite:1.0

# Anaconda
# Install miniconda
ARG CONDA_DIR=/opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p $CONDA_DIR

# Add conda to PATH
ENV PATH=$CONDA_DIR/bin:$PATH

RUN apt-get update && \
    apt-get install -y \
        clang-tidy clang-format \
        libblas3 liblapack3 liblapack-dev libblas-dev gfortran \
        libyaml-cpp-dev \
        libgl1 \
        doctest-dev && \
    rm -rf /var/lib/apt/lists/*

# create conda environments for python versions
ARG PY_VERSIONS="3.7 3.8 3.9 3.10 3.11"
ENV CONDA_ENV_PREFIX="commonroad-py"
RUN for PY_VERSION in $PY_VERSIONS; do conda create -n "$CONDA_ENV_PREFIX$PY_VERSION" python=$PY_VERSION; done
