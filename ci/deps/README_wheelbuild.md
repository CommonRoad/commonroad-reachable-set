# Content
This readme describes how to set up a docker to build python wheels for manylinux distributions.
- To create manylinux distributions (i.e. distributions for several linux systems) automatically, you need to use the python module **auditwheels**.
- However, choosing auditwheels runs into unsolvable problems, if some basic libraries are compiled on a too new linux system.
- Be aware that the first mistake is to choose a new linux distribution as docker base, thereby creating these problems

***

# General Steps

## 1. Creating a wheel-building docker
1. install docker as described on the docker homepage
2. create a new root directory on your system
3. create a dockerfile named **Dockerfile** in the root (see below)
4. run **docker build ./ -t IMAGENAME:TAG** to build the docker image
5. run **docker run --rm -it --network=host --shm-size=4gb -e DISPLAY=$DISPLAY IMAGENAME:TAG bash** to start the container

## 2. Using the created docker for building in gitlab CI/CD
1. Go to the gitlab repo -> Packages and registries -> Container Registry and click on the top right to CLI Commands to get the access address 
    - The general container address (something like docker login gitlab.lrz.de:5005)
    - Push address (without the docker push part)
2. use **sudo docker login CONTAINER_ADDRESS_FROM_STEP_2.1** to log into gitlab
3. use **sudo docker images** to find the name of your image (e.g. check the time it was built)
4. tag your docker image **sudo docker tag IMAGENAME PUSH_ADDRESS_FROM_STEP_1/IMAGENAME:TAG** according to your project standard, if you haven't done it already in step 1.6.
5. push the docker image **sudo docker push IMAGENAME PUSH_ADDRESS_FROM_STEP_1/IMAGENAME:TAG**. Do not use the suggested push command from gitlab.
6. in you browser, reload the container registry, the container should appear there now


## 3. Downloading the internal pip wheel
1. Make sure you **delete the current package, iff it has the same version as the one you want to create**
2. Click on your profile in the right corner -> edit profile -> generate token **with activated api rights**
3. In the gitlab repo go to **packages and registries/Package Registry**, choose the newly created package
4. Follow the pip install instructions in a conda env and use the token (NAME + Password) you created
5. Use the test_script.py to see whether the installation succeeded

***
# Dockerfile
- see the Dockerfile in this repo for an example
- to build the python wheels for many linux systems, you can use the image **quay.io/pypa/manylinux2014_x86_64**, but **!WARNING!**, this is CentOS based
- the libraries in CentOS are installed via **yum** and usually have **different names than their Ubuntu versions**

### General procedure
- resources: gitlab for manylinux: https://github.com/pypa/manylinux --> behind each image, the import (FROM blabla in Dockerfile) and the os is written
It is recommended to start with a Dockerfile that just imports the pypa manylinux container as base (see manylinux gitlab for address) and then activate the docker and test each command in there.

***
# Common Errors
1. Cmake has doesn't find context to non-included 3rd party libraries like Eigen3
    - You probably used the library name for Ubuntu and not CentOS --> google yum install for it
    - yaml-cpp-devel has a bug. You cannot simply install it with yum. Do the following: 
        - For now, yaml-cpp is disabled in the project requirements
        - This might also be related to the fact that there yaml-cpp is provided with conda on CentOS
2. apt-get doesnt seem to work properly
    - You are using a manylinux image which is based on CentOS7 but apt-get is for Ubuntu. Use **yum** instead.
3. auditwheel fails
    - You probably did not use the manylinux image as a base (FROM in the Dockerfile) or the image is too new
4. Weird docker login access errors
    - Check in the container registry the blue button for CLI Commands, if you copied the correct access information
5. When testing after pip install, some Python Modules seem to be missing
    - Check that in the CI.yaml, in the twine command of the push to internal register, you `push the dist/wheelhouse/*`