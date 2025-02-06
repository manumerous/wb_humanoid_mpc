ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE=ros:${ROS_DISTRO}-ros-base
FROM ${BASE_IMAGE} AS base

ARG USER_ID=1000
ARG GROUP_ID=1000
ARG WB_HUMANOID_MPC_DIR="/wb_humanoid_mpc_ws"
ARG PYTHON_VERSION=3.12
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# Install system dependencies, ROS, Python, and development tools in a single step
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release software-properties-common \
    locales \
    git \
    x11-apps \
    mesa-utils \
    gettext-base \
    && rm -rf /var/lib/apt/lists/*

COPY dependencies.txt /tmp/dependencies.txt

RUN apt-get update && \
    envsubst < /tmp/dependencies.txt | xargs apt-get install -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Set default shell to bash
SHELL ["/bin/bash", "-c"]

# Set up environment variables
ENV CMAKE_PREFIX_PATH="/opt/ros/${ROS_DISTRO}:${CMAKE_PREFIX_PATH}"
ENV DISPLAY=${DISPLAY} QT_X11_NO_MITSHM=1

# Set up user and workspace
WORKDIR ${WB_HUMANOID_MPC_DIR}
RUN mkdir -p ${WB_HUMANOID_MPC_DIR}/src/wb_humanoid_mpc

# Ensure correct UID/GID
RUN if ! getent group $GROUP_ID > /dev/null; then groupadd --gid $GROUP_ID devgroup; fi && \
    if ! getent passwd $USER_ID > /dev/null; then useradd --uid $USER_ID --gid $GROUP_ID --create-home devuser; fi && \
    chown -R $USER_ID:$GROUP_ID .

# Copy the Makefile and other necessary files
COPY . ${WB_HUMANOID_MPC_DIR}/src/wb_humanoid_mpc
