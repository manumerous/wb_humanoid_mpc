#!/usr/bin/env bash
set -euo pipefail

# Allow GUI applications
xhost +SI:localuser:root

# Generate Xauthority file for X11 forwarding
XAUTH=/tmp/.docker.xauth
if [ ! -f "${XAUTH}" ]; then
  touch "${XAUTH}"
  xauth nlist "${DISPLAY}" \
    | sed -e 's/^..../ffff/' \
    | xauth -f "${XAUTH}" nmerge -
  chmod a+r "${XAUTH}"
fi

# Host workspace root (hard‑coded)
HOST_WS="/home/mov/Documents/humanoid_mpc_ws"        # your workspace root
BUILD_WS="${HOST_WS}/build"
INSTALL_WS="${HOST_WS}/install"

# Run the container, mounting the entire workspace
# Bind mounts: src/, build/, and install/ persist on host

docker run --rm -it \
  --name wb-mpc-dev \
  --gpus all \
  --net host \
  --privileged \
  -u root \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY="${XAUTH}" \
  -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${XAUTH}:${XAUTH}:rw" \
  -v "${HOST_WS}:/wb_humanoid_mpc_ws:cached" \
  -v "${BUILD_WS}:/wb_humanoid_mpc_ws/build:cached" \
  -v "${INSTALL_WS}:/wb_humanoid_mpc_ws/install:cached" \
  --workdir /wb_humanoid_mpc_ws \
  wb-humanoid-mpc:dev \
  bash -c "chown -R ubuntu:ubuntu /wb_humanoid_mpc_ws/build /wb_humanoid_mpc_ws/install && exec su ubuntu -c bash"

echo "Done."

