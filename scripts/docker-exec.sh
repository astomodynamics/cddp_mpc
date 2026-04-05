#!/bin/bash
# Execute command in running cddp_mpc container

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_NAME="$(basename "$PROJECT_DIR")"

CONTAINER_ID="$(
    docker ps \
        --filter "name=^cddp-mpc-dev$" \
        --format '{{.ID}}' \
        | head -n1
)"

if [ -z "$CONTAINER_ID" ]; then
    CONTAINER_ID="$(
        docker ps \
            --filter "label=com.docker.compose.project=${PROJECT_NAME}" \
            --filter "label=com.docker.compose.service=cddp-mpc-dev" \
            --format '{{.ID}}' \
            | head -n1
    )"
fi

if [ -z "$CONTAINER_ID" ]; then
    echo "No running cddp-mpc dev container found."
    echo "Start one with: ./scripts/docker-run.sh bash"
    exit 1
fi

if [ "$#" -eq 0 ]; then
    set -- bash
fi

docker exec -it \
    -e ROS_HOME=/tmp/ros \
    -e ROS_LOG_DIR=/tmp/ros/log \
    "$CONTAINER_ID" /bin/bash /entrypoint.sh "$@"
