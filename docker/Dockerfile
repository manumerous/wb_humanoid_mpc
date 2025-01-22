ARG BASE_IMAGE=ubuntu:24.04

FROM ${BASE_IMAGE} AS base

ARG WB_HUMANOID_MPC_DIR="/wb_humanoid_mpc"

# Set environment variables for non-interactive installation and ROS distribution
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Update and install basic tools and dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Set up locales
RUN locale-gen en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Add the ROS2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 and additional ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-mcap-vendor \
    ros-${ROS_DISTRO}-interactive-markers \
    && rm -rf /var/lib/apt/lists/*

# Install additional development tools and libraries
RUN apt-get update && apt-get install -y \
    clang-format \
    make \
    build-essential \
    locales \
    libeigen3-dev \
    libglpk-dev \
    libboost-all-dev \
    libboost-filesystem-dev \
    libboost-log-dev \
    libglfw3-dev \
    ninja-build \
    ccache \
    python3-pygame \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# Install colcon (ROS2 build tool) and additional Python tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-argcomplete \
    python3-pip \
    && pip3 install -U setuptools \
    && rm -rf /var/lib/apt/lists/*

# Source ROS2 setup script and add it to the bashrc for convenience
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

WORKDIR ${WB_HUMANOID_MPC_DIR}

RUN make build-all
