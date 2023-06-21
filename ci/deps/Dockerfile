FROM continuumio/anaconda3:2023.03-1

RUN apt-get update && \
    apt-get install -y \
    git \
    build-essential \
    cmake \
    clang clang-tidy clang-format \
    libblas3 liblapack3 liblapack-dev libblas-dev gfortran \
    libboost-dev \
    libboost-thread-dev \
    libboost-test-dev \
    libboost-filesystem-dev \
    libeigen3-dev \
    libomp-dev \
    libcgal-dev \
    libyaml-cpp-dev \
    libgl1 \
    doctest-dev && \
    rm -rf /var/lib/apt/lists/*

RUN for PY_VERSION in 3.10 3.9 3.8 3.7; do conda create -n "cr$(echo $PY_VERSION | sed -e 's/\.//g')" python=$PY_VERSION; done

# install drivability checker
ARG CRDC_DIR="/dc"
ARG LOCAL_CRDC_DIR="./commonroad-drivability-checker"
WORKDIR $CRDC_DIR
COPY $LOCAL_CRDC_DIR/requirements.txt .
RUN for PY_ENV in cr310 cr39 cr38 cr37; do conda run -n $PY_ENV pip install -r ./requirements.txt; done
ARG BUILD_JOBS=4
COPY $LOCAL_CRDC_DIR .
RUN mkdir build.bak && \
    for PY_ENV in cr310 cr39 cr38 cr37; do conda run -n $PY_ENV bash build.sh -j $BUILD_JOBS --cgal -i && mv build/*-cpython-*/ build.bak; done && \
    rm -r build && \
    mv build.bak build
