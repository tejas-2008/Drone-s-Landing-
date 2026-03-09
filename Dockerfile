# ============================================================
# Base Image
# ============================================================
FROM ros:noetic-perception

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# ============================================================
# Create non-root user: tejas
# ============================================================
ARG USERNAME=tejas
ARG UID=1000
ARG GID=1000

RUN groupadd --gid ${GID} ${USERNAME} && \
    useradd --uid ${UID} --gid ${GID} -m ${USERNAME} && \
    usermod -aG sudo ${USERNAME}

ENV HOME=/home/${USERNAME}

# ============================================================
# System + ROS + Gazebo dependencies
# ============================================================
RUN apt-get update && apt-get install -y \
    sudo \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    nano \
    vim \
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    python3-wstool \
    python3-numpy \
    python3-opencv \
    libopencv-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    gazebo11 \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-plugins \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-camera-info-manager \
    ros-noetic-vision-opencv \
    ros-noetic-rqt \
    ros-noetic-rqt-image-view \
    ros-noetic-fiducial-msgs \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    libxcb-xinerama0 \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# Passwordless sudo
# ============================================================
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# ============================================================
# MAVROS GeographicLib datasets
# ============================================================
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && sudo chmod +x install_geographiclib_datasets.sh \
    && sudo ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

# ============================================================
# PX4 build dependencies
# ============================================================
RUN apt-get update && apt-get install -y \
    python3-jinja2 \
    python3-empy \
    python3-toml \
    python3-yaml \
    python3-dev \
    ninja-build \
    exiftool \
    protobuf-compiler \
    libxml2-utils \
    xsltproc \
    zip \
    unzip \
    ant \
    openjdk-11-jdk \
    clang \
    lld \
    gdb \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# Workspaces
ENV DEV_WS=${HOME}/dev_ws
ENV PX4_WS=${HOME}/px4_ws

RUN mkdir -p ${DEV_WS}/src ${PX4_WS} && \
    sudo chown -R ${USERNAME}:${USERNAME} ${HOME}

# ============================================================
# ros_entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh

# ============================================================
# rosdep (root only, once)
RUN rosdep init || true
RUN rosdep update

# ============================================================
# Copy PX4 source into image
COPY px4_ws/PX4-Autopilot ${PX4_WS}/PX4-Autopilot
RUN sudo chown -R ${USERNAME}:${USERNAME} ${PX4_WS}

# ============================================================
# Switch to user tejas
USER ${USERNAME}
WORKDIR ${PX4_WS}/PX4-Autopilot

# ============================================================
# PX4 Pre-build steps
RUN bash -c "\
    cd ${PX4_WS}/PX4-Autopilot && \
    git submodule update --init --recursive && \
    git fetch --all && \
    git checkout main && \
    python3 -m pip install --upgrade pip && \
    pip3 install -q empy jinja2 pyros-genmsg catkin_pkg && \
    pip3 install -q numpy --upgrade && \
    pip3 install -q toml pyyaml six && \
    pip3 install -q future \
"


# ============================================================
# Build PX4 SITL (Gazebo)
RUN bash -c "\
    source /opt/ros/noetic/setup.bash"
# ============================================================
# Entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash && cd ~/dev_ws"]
