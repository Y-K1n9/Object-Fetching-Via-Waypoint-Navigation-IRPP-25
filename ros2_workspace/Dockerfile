FROM osrf/ros:humble-desktop-full

# ── System dependencies ────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # TurtleBot3
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3-gazebo \
    # Nav2
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    # Gazebo 11
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    # CV bridge + ArUco
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    # TF2
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    # Python extras
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    # Utilities
    git \
    wget \
    vim \
    && rm -rf /var/lib/apt/lists/*

# ── Python packages ────────────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir opencv-contrib-python-headless

# ── Create non-root user 'ros' ─────────────────────────────────────────────────
RUN useradd -m -s /bin/bash -G sudo ros && \
    echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ── Workspace setup ────────────────────────────────────────────────────────────
RUN mkdir -p /workspaces/ros2_workspace/src
WORKDIR /workspaces/ros2_workspace

# Copy source into image (used when building stand-alone, not as dev container)
COPY src/ src/

# ── Build the package ──────────────────────────────────────────────────────────
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    export TURTLEBOT3_MODEL=waffle && \
    colcon build --symlink-install"

# ── Environment variables ──────────────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ros/.bashrc && \
    echo "source /workspaces/ros2_workspace/install/setup.bash 2>/dev/null || true" >> /home/ros/.bashrc && \
    echo "export TURTLEBOT3_MODEL=waffle" >> /home/ros/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> /home/ros/.bashrc

RUN chown -R ros:ros /workspaces/ros2_workspace

USER ros