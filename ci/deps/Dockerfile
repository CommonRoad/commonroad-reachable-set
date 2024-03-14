FROM debian:bullseye-slim

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && \
    apt-get install -y \
        build-essential git pkg-config \
        clang ninja-build \
        ca-certificates wget \
        gcovr doxygen graphviz cppcheck \
        libomp-dev \
        libeigen3-dev \
        libboost-all-dev \
        libcgal-dev \
        libblas3 liblapack3 liblapack-dev libblas-dev gfortran \
        libyaml-cpp-dev \
        libgl1 \
        doctest-dev && \
    rm -rf /var/lib/apt/lists/*

# Install newer version of CMake than provided by the standard repos
ARG CMAKE_VERSION=3.28.3
RUN wget "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-Linux-x86_64.sh" \
    -q -O /tmp/cmake-install.sh && \
    chmod u+x /tmp/cmake-install.sh && \
    mkdir /opt/cmake && \
    /tmp/cmake-install.sh --skip-license --prefix=/opt/cmake && \
    rm /tmp/cmake-install.sh
ENV PATH="/opt/cmake/bin:${PATH}"
