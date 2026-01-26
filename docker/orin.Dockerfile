FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release ca-certificates \
    build-essential cmake git \
    python3 python3-pip python3-colcon-common-extensions python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# micro XRCE-DDS agent (snap is common on Ubuntu, but in docker we build from apt when available)
# If you prefer snap on host, run agent natively instead of in container.
RUN apt-get update && apt-get install -y --no-install-recommends \
    micro-xrce-dds-agent \
    && rm -rf /var/lib/apt/lists/* || true

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /work
