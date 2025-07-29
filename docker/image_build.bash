#!/usr/bin/env bash
# image_build.bash - Build the WB Humanoid MPC Docker image
# Reflects the "build" section of .devcontainer/devcontainer.json
set -euo pipefail

# Paths (relative to this script)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Assumes this script lives in your docker/ folder next to Dockerfile
DOCKERFILE="${SCRIPT_DIR}/Dockerfile"
CONTEXT="${SCRIPT_DIR}/.."
TARGET="base"

# Build arguments (from devcontainer.json)
: "# Workspace directory inside container"
WB_HUMANOID_MPC_DIR="/wb_humanoid_mpc_ws"
PYTHON_VERSION="3.12"
USER_ID="$(id -u)"
GROUP_ID="$(id -g)"
GIT_USER_NAME="$(git config --global user.name || echo '')"
GIT_USER_EMAIL="$(git config --global user.email || echo '')"

# Image tag
IMAGE_TAG="wb-humanoid-mpc:dev"

# Build command
docker build \
  --file "${DOCKERFILE}" \
  --target "${TARGET}" \
  --build-arg WB_HUMANOID_MPC_DIR="${WB_HUMANOID_MPC_DIR}" \
  --build-arg PYTHON_VERSION="${PYTHON_VERSION}" \
  --build-arg USER_ID="${USER_ID}" \
  --build-arg GROUP_ID="${GROUP_ID}" \
  --build-arg GIT_USER_NAME="${GIT_USER_NAME}" \
  --build-arg GIT_USER_EMAIL="${GIT_USER_EMAIL}" \
  --tag "${IMAGE_TAG}" \
  "${CONTEXT}"

echo "Built image: ${IMAGE_TAG}"