# This shell script is used to automatically build the manylinux container for wheel-building and push it to the container registry
# -------------------------------------------------------------------------------------------------------------------------------

# Login to docker -> See packages and container -> container registry -> cli commands
sudo docker login gitlab.lrz.de:5005

# get current version of development branch of the drivability checker
git clone -b development --recurse-submodules --single-branch --depth 1 git@gitlab.lrz.de:cps/commonroad-drivability-checker.git commonroad-drivability-checker

# build the wheelbuilding-container
sudo docker build --build-arg LOCAL_CRDC_DIR="./commonroad-drivability-checker" -t gitlab.lrz.de:5005/cps/commonroad-reachable-set/manylinux_wheelbuild:latest .

# remove the drivability checker repo after container building
rm -rf commonroad-drivability-checker

# push the container to the container registry
sudo docker push gitlab.lrz.de:5005/cps/commonroad-reachable-set/manylinux_wheelbuild:latest

# logout
sudo docker logout gitlab.lrz.de:5005

# --> Now, you have to manually trigger the pipeline. It is possible that older versions of the package with the same tag have to be deleted manually.
