# Use the minimal ROS Galactic image
FROM ros:galactic-ros-core

# Minimal setup
RUN apt update && \
    apt install -y locales lsb-release && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-colcon-common-extensions \
    build-essential \
    git \
    ros-galactic-xacro \
    ros-galactic-gazebo-ros\
    ros-galactic-gazebo-plugins \
    ros-galactic-ament-cmake \
    ros-galactic-robot-state-publisher \
    ros-galactic-joint-state-publisher \
    ros-galactic-navigation2 \
    ros-galactic-nav2-bringup \
    ros-galactic-cartographer-ros \
    ros-galactic-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Set the default shell to bash to ensure sourcing works correctly
SHELL ["/bin/bash", "-c"]

# Git clone the waypoint test into the workspace
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/tianyaohu/ros2_tortoisebot_GTest.git

# Entrypoint bash
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/galactic/setup.bash && exec bash"]
