FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# ── System packages ──
RUN apt-get update && apt-get install -y \
    sudo \
    locales \
    git \
    curl \
    wget \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    bash-completion \
    python3-pip \
    python3-opencv \
    python3-yaml \
    # TurtleBot3 packages
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-turtlebot3-simulations \
    # Navigation2 stack
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-nav2-common \
    # Gazebo ROS integration
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    # Vision / ArUco
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    # TF2
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    # Robot state publisher
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Install OpenCV contrib (for ArUco) via pip
RUN pip3 install opencv-contrib-python-headless

RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# ── TurtleBot3 model env var ──
ENV TURTLEBOT3_MODEL=waffle

# ── Create non-root user ──
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && usermod -aG sudo,dialout ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# ── Source ROS 2 and TurtleBot3 setup in every shell ──
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
    && echo "export TURTLEBOT3_MODEL=waffle" >> /etc/bash.bashrc \
    && echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> /etc/bash.bashrc

WORKDIR /home/${USERNAME}/ws
RUN mkdir -p src && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}
USER ${USERNAME}

CMD ["bash"]
