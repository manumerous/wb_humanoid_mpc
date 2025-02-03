ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE} AS base

ARG USER_ID=1000
ARG GROUP_ID=1000
ARG WB_HUMANOID_MPC_DIR="/wb_humanoid_mpc_ws"
ARG PYTHON_VERSION=3.12
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install system dependencies, ROS, Python, and development tools in a single step
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release locales software-properties-common \
    git build-essential clang-format make ninja-build \
    python3-pygame libeigen3-dev libglpk-dev libboost-all-dev \
    libboost-filesystem-dev libboost-log-dev libglfw3-dev \
    x11-apps mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Set up locales
RUN locale-gen en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8 LANGUAGE en_US:en LC_ALL en_US.UTF-8

# Install Python and pip
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python${PYTHON_VERSION} python${PYTHON_VERSION}-dev python${PYTHON_VERSION}-tk libpython${PYTHON_VERSION}-dev \
    python3-pip python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*
RUN ln -sf /usr/bin/python${PYTHON_VERSION} /usr/bin/python3
RUN python3 -m pip install --break-system-packages --no-cache-dir --upgrade setuptools cython catkin_pkg colcon-common-extensions

# Add ROS2 repository and install core packages
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    ros-${ROS_DISTRO}-joint-state-publisher-gui ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-mcap-vendor ros-${ROS_DISTRO}-interactive-markers \
    ros-${ROS_DISTRO}-std-msgs ros-${ROS_DISTRO}-pinocchio \
    && rm -rf /var/lib/apt/lists/*

# Increase Git buffer size
RUN git config --global http.postBuffer 1048576000

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
