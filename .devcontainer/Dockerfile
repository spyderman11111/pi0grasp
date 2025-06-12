# Base image for ROS 2 Jazzy with development tools
# FROM osrf/ros:jazzy-desktop-full AS base
# Using osrf/ros:jazzy-desktop-full pinned to a specific digest for reproducibility
FROM osrf/ros@sha256:de577f404a2190dbc2ebdc67477a5aab69b5525099ab2bf4f8bd238f06fcdf5e AS base

# Fix ROS key and apt sources before installing packages
RUN rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    rm /etc/apt/sources.list.d/ros2-latest.list && \
    apt-key del "C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654" || true && \
    apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install -y curl && \
    ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    apt-get install -y /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

# Install necessary development tools and dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-dev \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    g++ \
    curl \
    wget \
    lsb-release \
    gnupg2 \
    openssh-client \
    libusb-1.0-0-dev \
    ros-jazzy-tf-transformations \
    ros-jazzy-rosbridge-server \
    gedit \
    ssh-client \
    iputils-ping \
    net-tools \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    tree \
    ros-jazzy-ur \
    iptables \
    usbutils \
&& rm -rf /var/lib/apt/lists/*

# FROM base AS ur_driver

# WORKDIR /root/ws_ur_driver
# RUN mkdir -p src
# RUN git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver && \
#     cd src/Universal_Robots_ROS2_Driver && \
#     git checkout 34e17a3a1892a814c207cf31e7214c6320909c33 # this is the stable version

# WORKDIR /root/ws_ur_driver
# RUN vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.jazzy.repos
# RUN apt-get update && apt-get install -y\
#     python3-colcon-common-extensions \
#     ros-jazzy-ament-cmake \
#     python3-vcstool
# RUN rosdep update && rosdep install --ignore-src --from-paths src -y -r
# RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

FROM base AS final

RUN apt-get update && \
    apt-get install -y curl && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Micromamba
RUN curl -L micro.mamba.pm/install.sh | bash

# Clean up apt cache to reduce image size
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
# RUN echo "source /root/ws_ur_driver/install/setup.bash" >> ~/.bashrc

# Set the entrypoint to bash
CMD ["/bin/bash"]
