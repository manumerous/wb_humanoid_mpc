ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE} AS base

ARG USER_ID=1000
ARG GROUP_ID=1000

ARG WB_HUMANOID_MPC_DIR="/wb_humanoid_mpc_ws"
ARG PYTHON_VERSION=3.12

# Switch to non-root user
USER $USER_ID:$GROUP_ID

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
    software-properties-common \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install Python 3.12 explicitly
RUN add-apt-repository -y ppa:deadsnakes/ppa && apt-get update && apt-get install -y \
    python${PYTHON_VERSION} \
    python${PYTHON_VERSION}-dev \
    python${PYTHON_VERSION}-tk \
    libpython${PYTHON_VERSION}-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Ensure Python 3.12 is the default
RUN ln -sf /usr/bin/python${PYTHON_VERSION} /usr/bin/python3

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
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-mcap-vendor \
    ros-${ROS_DISTRO}-interactive-markers \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-pinocchio \
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
    && rm -rf /var/lib/apt/lists/*

# Install colcon (ROS2 build tool) and additional Python tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip and install required Python packages globally
RUN python3 -m pip install --break-system-packages --no-deps --upgrade setuptools \
    && python3 -m pip install --break-system-packages --no-deps --no-cache-dir --force-reinstall cython catkin_pkg

# Verify NumPy installation
RUN python3 -c "import numpy; print(numpy.__version__)"

# Increase Git buffer size
RUN git config --global http.postBuffer 1048576000

# Source ROS2 setup script and add it to the bashrc for convenience
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

WORKDIR ${WB_HUMANOID_MPC_DIR}

RUN mkdir -p ${WB_HUMANOID_MPC_DIR}/src/wb_humanoid_mpc

# Ensure Git recognizes the repository as safe
RUN git config --global --add safe.directory ${WB_HUMANOID_MPC_DIR}/src/wb_humanoid_mpc

# Ensure correct UID/GID are set and valid
RUN if ! getent group $GROUP_ID > /dev/null; then groupadd --gid $GROUP_ID devgroup; fi && \
    if ! getent passwd $USER_ID > /dev/null; then useradd --uid $USER_ID --gid $GROUP_ID --create-home devuser; fi && \
    chown -R $USER_ID:$GROUP_ID .

# Copy the Makefile and other necessary files to the working directory
COPY . ${WB_HUMANOID_MPC_DIR}/src/wb_humanoid_mpc

# Set default shell to bash
SHELL ["/bin/bash", "-c"]

# Set CMAKE_PREFIX_PATH to include ROS2 installation directory
ENV CMAKE_PREFIX_PATH="/opt/ros/${ROS_DISTRO}:${CMAKE_PREFIX_PATH}"

# Source ROS2 setup script before running commands
RUN source /opt/ros/${ROS_DISTRO}/setup.bash

# Install X11 dependencies for GUI applications
RUN apt-get update && apt-get install -y \
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Set environment variables for X11 forwarding
ENV DISPLAY=${DISPLAY}
ENV QT_X11_NO_MITSHM=1

# cd ${WB_HUMANOID_MPC_DIR}/src/wb_humanoid_mpc && make build-all
