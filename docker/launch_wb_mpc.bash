#!/usr/bin/env bash
#
# Usage:
#
# $ cd ~/your_colcon_ws/src/wb_humanoid_mpc/docker
# $ ./launch_wb_mpc.bash    # Launch the WB Humanoid MPC Docker container
#
# (Cross reference this file with the "run" section of ../.devcontainer/devcontainer.json)
#
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

# ROOT_COLCON_WS 
HOST_WS="$(realpath "${PWD}/../../..")"

# Run the container, mounting the entire workspace
docker run --rm -it \
  --name wb-mpc-dev \
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
  --workdir /wb_humanoid_mpc_ws \
  wb-humanoid-mpc:dev \
  bash

echo "Done."

