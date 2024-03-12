## Updating the Docker Image for CI

If we need any new dependencies, we need to update the Docker image that is used for CI.

1. Update the `Dockerfile` in this directory with the new dependencies
2. Build the Docker image locally
    ```bash
    docker build -t gitlab.lrz.de:5005/cps/commonroad-reachable-set/deps:<TAG> .
    ```
    > **Important:** Use the same tag as the one in the `.gitlab-ci.yml` file.
3. Login to the GitLab container registry
    ```bash
    docker login gitlab.lrz.de:5005
    ```
4. Push the image to the GitLab container registry
    ```bash
    docker push gitlab.lrz.de:5005/cps/commonroad-reachable-set/deps:<TAG>
    ```
