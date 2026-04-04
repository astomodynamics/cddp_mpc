#!/bin/bash
# Build Docker images for cddp_mpc

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR"

echo "Building cddp-mpc development image..."

export USER_UID=$(id -u)
export USER_GID=$(id -g)

docker compose -f docker-compose.yml build cddp-mpc-dev

echo ""
echo "Build complete!"
echo "Run with: ./scripts/docker-run.sh"
