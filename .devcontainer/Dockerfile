FROM ros:foxy-ros-base-focal

# using bash
SHELL ["/bin/bash", "-c"]

# remove debconf errors (must run before any apt-get calls)

RUN echo "debconf debconf/frontend select Noninteractive" | debconf-set-selections

# install needed linux packages
# apt-get   https://wiki.ubuntuusers.de/apt/apt-get/

# -y yes to all displayed questions 
RUN apt-get update \
        && apt-get upgrade -y \
        && apt-get install -y apt-utils \
        software-properties-common \
        # Tools
        git \
        nodejs \ 
        openssh-client \ 
        # Driver lidar
        libudev-dev \
        libboost-all-dev \
        # CLI editors
        nano \
        vim \
        terminator \
        # Python development headers and RPi GPIO library
        python3-dev \
        python3-rpi.gpio

ARG USER
ARG TOKEN

# Set up stage_2025 workspace
WORKDIR /home
RUN  git clone https://gitlab.com/Ragnar711/stage_2025.git 

# docker build --build-arg USER="gitlab-username" --build-arg TOKEN="your-personal-gitlab-token" -t "name_image"

WORKDIR /home/stage_2025
# build the stage_2025 workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
        && colcon build --symlink-install

# write source commands to .bashrc -> no need to source afterwards
RUN echo "source /home/stage_2025/install/setup.bash" >> ~/.bashrc
