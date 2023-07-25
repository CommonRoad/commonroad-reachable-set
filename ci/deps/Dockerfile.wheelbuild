##########################################################################################################
#
# Steps:
#   1. git clone drivability checker dev branch in root
#   2. write docker file and call it Dockerfile
#   3. in root dir, run: sudo docker build ./ -t IMAGENAME
#   4. in root dir, run: sudo docker run --rm -it --network=host --shm-size=4gb -e DISPLAY=$DISPLAY manylinux:latest bash; and check inside container if everything is as expected
#   5. use: sudo docker login CONTAINER_ADRESS_FROM_STEP_1 to log into gitlab
#   6. use: sudo docker images to find the name of your image (e.g. check the time it was built)
#   7. tag your docker image: sudo docker tag IMAGENAME PUSH_ADRESS_FROM_STEP_1/IMAGENAME:TAG
#   8. push the docker iamge: sudo docker push IMAGENAME PUSH_ADRESS_FROM_STEP_1/IMAGENAME:TAG
#   9. Use the image for wheelbuilding --> see ci/wheelbuilding/.gitlab-ci.yml and ci/wheelbuilding/README
#
#
##########################################################################################################


FROM quay.io/pypa/manylinux2014_x86_64

# WARNING!: manylinux is CentOs based and not Ubuntu!

# install enterprice linux
RUN yum update \
    && yum install -y epel-release \ 
    && yum install -y wget \
    && yum install -y nano


# Install external third-party libraries
# !WARNING!: These libraries are named diffently than under Ubuntu

# libeigen3
RUN yum update \
    && yum install -y eigen3-devel

# libboost
RUN yum update \
    && wget https://boostorg.jfrog.io/artifactory/main/release/1.65.1/source/boost_1_65_1.tar.gz \
    && tar -xzf boost_1_* \
    && cd boost_1_* \
    && ./bootstrap.sh \
    && ./b2 install --with-thread --with-test --with-filesystem


# libyaml-cpp
# There is a problem with yampl-cpp under Centos which is not found correctly
RUN yum update \
    && yum install -y yaml-cpp-devel

# libomp-dev
RUN yum update \
    && yum install -y llvm-toolset-7-libomp-devel



# Miniconda
# Install miniconda
ENV CONDA_DIR /opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda

# Put conda in path so we can use conda activate
ENV PATH=$CONDA_DIR/bin:$PATH


# create conda envs for python 3.7 to 3.10
RUN for PY_VERSION in 3.10 3.9 3.8 3.7; do conda create -n "cr$(echo $PY_VERSION | sed -e 's/\.//g')" python=$PY_VERSION; done

# install drivability checker
ARG CRDC_DIR="/dc"
ARG LOCAL_CRDC_DIR="./commonroad-drivability-checker"
WORKDIR $CRDC_DIR
COPY $LOCAL_CRDC_DIR/requirements.txt .
COPY $LOCAL_CRDC_DIR .
RUN for PY_ENV in cr310 cr39 cr38 cr37; do conda run -n $PY_ENV pip install -r ./requirements.txt; done
ARG BUILD_JOBS=4
RUN mkdir build.bak && \
    for PY_ENV in cr310 cr39 cr38 cr37; do conda run -n $PY_ENV bash build.sh -j $BUILD_JOBS --cgal -i && mv build/*-cpython-*/ build.bak; done && \
    rm -r build && \
    mv build.bak build


# lib is called lib64 in centos but our CMakeList wants lib as dir name .... perhaps copying lib64 to lib should suffice ? --> yes
RUN cd build/ && \ 
    for f in *; do if [ -d "$f" ]; then if [[ "$f" =~ "temp" ]]; then cp -r $f/dist/lib64/ $f/dist/lib; fi; fi; done



# yaml-cpp is not found correctly by CMakeList

