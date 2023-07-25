docker login gitlab.lrz.de:5005
git clone -b development --recurse-submodules --single-branch --depth 1 git@gitlab.lrz.de:cps/commonroad-drivability-checker.git dc
docker build --build-arg LOCAL_CRDC_DIR="./dc" -t gitlab.lrz.de:5005/cps/commonroad-reachable-set/deps:ci .
rm -rf dc
docker push gitlab.lrz.de:5005/cps/commonroad-reachable-set/deps:ci
docker logout gitlab.lrz.de:5005
