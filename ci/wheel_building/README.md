# Content
This readme describes how to setup a docker to build python wheels for manylinux distributions.
- To create manylinux distributions (i.e. distributions for several linux systems) automatically, you need to use the python module **auditwheels**.
- However, choosing auditwheels runs into unsolvable problems, if some basic libraries are compiled on a too new linux system.
- Be aware that the first mistake is to choose a new linux distribution as docker base, thereby creating these problems


# Creating a wheel-building docker
1. install docker as described on the docker homepage
2. create a new root directory on your system
3. git clone commonroad-drivability-checker into the root
4. go into the commonroad-drivability-checker dir and run **git submodule update --init --recursive**. Then go back to the root 
5. create a dockerfile named **Dockerfile** in the root (see below)
6. run **sudo docker build ./ IMAGENAME:TAG** to build docker
7. run **sudo docker run --rm -it --network=host --shm-size=4gb -e DISPLAY=$DISPLAY IMAGENAME:TAG bash** to start the container

# Using the created docker for building in gitlab CI/CD
1. Got to the gitlab repo -> Packages and registries -> Container Registry and click on the top right to CLI Commands to get the access adress 
    - The general container adress (something like registry.gitlab.com:5005)
    - Push adress (whithout the docker push part)
2. use **sudo docker login CONTAINER_ADRESS_FROM_STEP_1** to log into gitlab
3. use **sudo docker images** to find the name of your image (e.g. check the time it was built)
4. tag your docker image **sudo docker tag IMAGENAME PUSH_ADRESS_FROM_STEP_1/IMAGENAME:TAG**
5. push the docker iamge **sudo docker push IMAGENAME PUSH_ADRESS_FROM_STEP_1/IMAGENAME:TAG**
6. in you browser, reload the container registry, the container should appear there now


# Dockerfile
- see the Dockerfile in this repo for an example
- to build the python wheels for many linux systems, you can use the image **quay.io/pypa/manylinux2014_x86_64**, but **!WARNING!**, this is CentOS based
- the libraries in CentOS are installed via **yum** and usually have **different names than their Ubuntu versions**


# Gitlab CI via .gitlab-ci.yaml
- be aware that the requirements to be automatically installed are specified in **setup.py** and **not in requirements.txt**

### General procedure
- resources: gitlab for manylinux: https://github.com/pypa/manylinux --> behind each image, the import (FROM blabla in Dockerfile) and the os is written
It is recommended to start with a Dockerfile that just imports the pypa manylinux container as base (see manylinux gitlab for adress) and then activate the docker and test each command in there.


# Common Errors
1. Cmake says for included 3rd party libraries: `NAME  is not an existing non-empty directory.  Please specify one of: etc.... `
    - You probably forgot to run **git submodule update --init --recursive** in the cloned commonroad-drivability checker root
2. Cmake has doesn't find context to non-included 3rd party libraries like Eigenlib3
    - You probably used the library name for Ubuntu and not CentOS --> google yum install for it
    - yaml-cpp-devel has a bug. You cannot simply install it with yum. Do the following: 
        - NO CLUE
3. apt-get doesnt seem to work properly
    - You are using a manylinux image which is based on CentOS7 but apt-get is for Ubuntu. Use **yum** instead.
4. auditwheel fails
    - You probable did not use the manylinux image as a base (FROM in the Dockerfile) or the image is to new
5. Weird docker login access errors
    - Check in the container registry the blue button for CLI Commands, if you copied the correct access information
6. When testing after pip install, some Python Modules seem to be missing
    - Check that in the CI.yaml, in the twine command of the push to internal register, you `push the dist/wheelhouse/*`
7. Some weird internal cpp calls with seemingly wrong interfaces, e.g. wrong interface of curviliniear_system function
    - you mixed up installations (for example of the drivability checker via setup.py and reach via manylinux wheels)
8. Not all requirements from requirement.txt are installed automatically
    - for a wheel installation, you have to write them in **setup.py** not in requirements.txt