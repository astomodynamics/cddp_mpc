#!/bin/bash
# Run cddp_mpc development container

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
CONTAINER_NAME="cddp-mpc-dev"

cd "$PROJECT_DIR"

export USER_UID=$(id -u)
export USER_GID=$(id -g)

xhost "+si:localuser:$(id -un)" 2>/dev/null || true

if [ "$#" -eq 0 ]; then
    set -- bash
fi

if docker ps --filter "name=^${CONTAINER_NAME}$" --format '{{.ID}}' | grep -q .; then
    echo "Reusing running ${CONTAINER_NAME} container..."
else
    echo "Starting ${CONTAINER_NAME} container..."
    docker compose -f docker-compose.yml up -d cddp-mpc-dev
fi

docker exec -it \
    -e ROS_HOME=/tmp/ros \
    -e ROS_LOG_DIR=/tmp/ros/log \
    "${CONTAINER_NAME}" /bin/bash /entrypoint.sh "$@"
