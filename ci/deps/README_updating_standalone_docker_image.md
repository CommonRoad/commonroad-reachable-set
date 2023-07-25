## Updating the Docker Image for CI

When the drivability checker changes or if we need any new dependencies, we need to update the Docker image that is used for CI.

1. Update the Dockerfile in this directory with the new dependencies
2. Clone the [drivability checker](https://gitlab.lrz.de/cps/commonroad-drivability-checker) repository into `dc`
    ```bash
    git clone -b $BRANCH_NAME --recurse-submodules --single-branch --depth 1 git@gitlab.lrz.de:cps/commonroad-drivability-checker.git dc
    ```
    This will clone just the branch `BRANCH_NAME` (usually this should be `development`) of the drivability checker without fetching the history (`--depth 1`) and initialize its git submodules.
3. Build the Docker image locally
    ```bash
    docker build --build-arg LOCAL_CRDC_DIR="./dc" -t gitlab.lrz.de:5005/cps/commonroad-reachable-set/deps:ci .
    ```
    > **Important:** Use the same tag as the one in the `.gitlab-ci.yml` file.
4. Login to the GitLab container registry
    ```bash
    docker login gitlab.lrz.de:5005
    ```
5. Push the image to the GitLab container registry
    ```bash
    docker push gitlab.lrz.de:5005/cps/commonroad-reachable-set/deps:ci
    ```
