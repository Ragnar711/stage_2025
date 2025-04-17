FROM ros:foxy-ros-base-focal

# using bash
SHELL ["/bin/bash", "-c"]

# remove debconf errors (must run before any apt-get calls)

RUN echo "debconf debconf/frontend select Noninteractive" | debconf-set-selections && apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y apt-utils \
    git \
    libboost-all-dev \
    libudev-dev \
    nano \
    openssh-client \
    python3-dev \
    python3-pip \
    python3-smbus \
    software-properties-common \
    terminator \
    vim \
    && pip3 install \
        RPi.GPIO \
        adafruit-circuitpython-mcp4725 \
        flask \
        mpu6050-raspberrypi \
        pandas \
        requests \
        smbus2 \
        sseclient-py \
        waitress \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


# Set up stage_2025 workspace
WORKDIR /home

RUN  git clone https://gitlab.com/land_drone/stage_2025.git 

# docker build --build-arg USER="gitlab-username" --build-arg TOKEN="your-personal-gitlab-token" -t "name_image"
WORKDIR /home/stage_2025

# build the stage_2025 workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
        && colcon build --symlink-install \
        && echo "source /home/stage_2025/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
