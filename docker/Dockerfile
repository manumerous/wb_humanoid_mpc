ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE=ros:${ROS_DISTRO}-ros-core

FROM ${BASE_IMAGE} AS base

ARG WB_HUMANOID_MPC_DIR="/wb_humanoid_mpc"

# Set environment variables for non-interactive installation and ROS distribution
ENV DEBIAN_FRONTEND=noninteractive

# Install Python and essential packages
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    build-essential \
    python3-venv \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Set up locales
RUN locale-gen en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Install additional development tools and libraries
RUN apt-get update && apt-get install -y \
    clang-format \
    make \
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

RUN apt-get update && apt-get install -y \
python3-colcon-common-extensions \
python3-argcomplete \
python3-pip \
&& rm -rf /var/lib/apt/lists/*

# Create a virtual environment
RUN python3 -m venv /opt/venv

# Make sure to use the virtual environment by default
ENV PATH="/opt/venv/bin:$PATH"

# Upgrade pip, setuptools, and wheel inside the virtual environment
RUN pip install --upgrade pip setuptools wheel

# Install numpy in the virtual environment
RUN pip install numpy

# Install colcon (ROS2 build tool) and other Python tools in the virtual environment
# RUN pip install python3-colcon-common-extensions python3-argcomplete

# Source ROS2 setup script and add it to the bashrc for convenience
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Set the working directory
WORKDIR ${WB_HUMANOID_MPC_DIR}

# Copy the entire project directory to the container
COPY . ${WB_HUMANOID_MPC_DIR}

# Optionally run the build-all target (commented out if interactive build is needed)
# RUN make build-all
