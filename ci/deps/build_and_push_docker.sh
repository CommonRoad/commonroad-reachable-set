#! /bin/bash

# This shell script builds the Docker image for the CI pipeline and pushes it to the GitLab container registry
# ------------------------------------------------------------------------------------------------------------

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <TAG>"
    echo "Example: $0 1.0"
    exit 1
fi

# Information
echo "Starting to build and push the containers for commonroad-reachable-set. This may take some time."
echo "You have to be in the docker group to run this script successfully."
echo -e "\n  -- LRZ GitLab login --"

# Docker gitlab login
docker login gitlab.lrz.de:5005

# Build Docker image
docker build -t gitlab.lrz.de:5005/cps/commonroad-reachable-set/deps:$1 .

# Push Docker image to GitLab registry
docker push gitlab.lrz.de:5005/cps/commonroad-reachable-set/deps:$1

# Docker gitlab logout
docker logout gitlab.lrz.de:5005

echo -e "\n Now, you have to manually trigger the build and deploy stage in the pipeline. It is possible that older versions of the package with the same tag have to be deleted manually."
