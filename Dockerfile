# syntax=docker/dockerfile:1.7
# Ubuntu 24.04 (Noble) + ROS 2 Jazzy

FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml \
    ROS_DOMAIN_ID=0

SHELL ["/bin/bash", "-c"]

# Keep downloaded .debs so the apt cache mounts below can reuse them.
RUN rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Core stack: MoveIt 2 + ros2_control + Foxglove. Large layer — leave it alone
# and add new packages to the layer below instead.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rmw-fastrtps-cpp \
    ros-jazzy-topic-tools \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-moveit \
    ros-jazzy-moveit-resources \
    ros-jazzy-moveit-configs-utils \
    ros-jazzy-moveit-ros-planning-interface \
    ros-jazzy-moveit-ros-perception

# Extras: add new apt packages here so the core layer stays cached.
# Periodically fold them into the core layer.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    tmux

# trimesh loads collision meshes for the planning scene. Noble ships numpy
# 1.26, which trimesh 4.x supports. Ubuntu 24.04 marks the system Python as
# externally managed (PEP 668), so pip needs --break-system-packages.
RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m pip install --break-system-packages 'trimesh>=4.0,<5'

# Small, frequently edited bits last.
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "alias foxglove='ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8765'" >> /root/.bashrc

# ISAAC SIM BRIDGE: Fast DDS UDP profile
COPY config/fastdds.xml /root/.ros/fastdds.xml

# ENTRY
# /ros_entrypoint.sh comes from the base image and sources /opt/ros/$ROS_DISTRO.
# Code is mounted at run time (see README), not copied in.
WORKDIR /ros2_ws
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
