
# This shell script is used to automatically build the containers for the cpp standalone and the python wheelbuild and push it to the container registry
# ------------------------------------------------------------------------------------------------------------------------------------------------------

# Information
echo "Starting to build and push the containers for commonroad-reachable-set. This takes about 30 minutes."
echo "You have to be in the docker group to run this script successfully."
echo -e "\n  -- lrz gitlab login --"

# Docker gitlab login
docker login gitlab.lrz.de:5005

# Clone development branch of cr-drivability-checker
git clone -b development --recurse-submodules --single-branch --depth 1 git@gitlab.lrz.de:cps/commonroad-drivability-checker.git commonroad-drivability-checker

# Create cpp standalone and push to container registry
docker build --build-arg LOCAL_CRDC_DIR="./commonroad-drivability-checker" -f Dockerfile.standalone -t gitlab.lrz.de:5005/cps/commonroad-reachable-set/deps:ci .
docker push gitlab.lrz.de:5005/cps/commonroad-reachable-set/deps:ci


# Create pypi wheelbuilding container and push to container registry
docker build --build-arg LOCAL_CRDC_DIR="./commonroad-drivability-checker" -f Dockerfile.wheelbuild -t gitlab.lrz.de:5005/cps/commonroad-reachable-set/manylinux_wheelbuild:latest .
docker push gitlab.lrz.de:5005/cps/commonroad-reachable-set/manylinux_wheelbuild:latest


# Remove cloned drivabilty checker
rm -rf commonroad-drivability-checker

# docker gitlab logout
docker logout gitlab.lrz.de:5005


# --> Now, you have to manually trigger the build and deploy stage in the  pipeline. It is possible that older versions of the package with the same tag have to be deleted manually.
echo -e "\n Now, you have to manually trigger the build and deploy stage in the  pipeline. It is possible that older versions of the package with the same tag have to be deleted manually."
