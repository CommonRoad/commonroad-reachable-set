#! /bin/bash

# This shell script is used to automatically build the containers for the cpp standalone and the python wheelbuild and push it to the container registry
# ------------------------------------------------------------------------------------------------------------------------------------------------------

# Information
echo "Starting to build and push the containers for commonroad-reachable-set. This may take some time."
echo "You have to be in the docker group to run this script successfully."
echo -e "\n  -- lrz gitlab login --"

# Docker gitlab login
docker login gitlab.lrz.de:5005

# Create cpp standalone and push to container registry
docker build -f Dockerfile.standalone -t gitlab.lrz.de:5005/cps/commonroad-reachable-set/deps:ci .
docker push gitlab.lrz.de:5005/cps/commonroad-reachable-set/deps:ci


# Create pypi wheelbuilding container and push to container registry
docker build -f Dockerfile.wheelbuild -t gitlab.lrz.de:5005/cps/commonroad-reachable-set/manylinux_wheelbuild:latest .
docker push gitlab.lrz.de:5005/cps/commonroad-reachable-set/manylinux_wheelbuild:latest

# docker gitlab logout
docker logout gitlab.lrz.de:5005

echo -e "\n Now, you have to manually trigger the build and deploy stage in the pipeline. It is possible that older versions of the package with the same tag have to be deleted manually."
