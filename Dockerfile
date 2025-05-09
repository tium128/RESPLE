# ROS base
FROM osrf/ros:humble-desktop-full

# Choose bash shell
SHELL ["/bin/bash", "-c"]

# Environment Variables
ENV HOME=/root

# Update apt
RUN apt update

# Install git and SSH client
RUN apt install -y git openssh-client

# Add public github and bitbucket keys
#RUN mkdir -p -m 0600 $HOME/.ssh
#RUN ssh-keyscan github.com >> $HOME/.ssh/known_hosts
#RUN ssh-keyscan bitbucket.org >> $HOME/.ssh/known_hosts

# Install system dependencies
RUN apt update && apt install -y \
    libeigen3-dev \
    libomp-dev \
    libpcl-dev \
    ros-humble-pcl*

# Install mcap support
RUN apt install -y ros-humble-rosbag2-storage-mcap

# Install gdb support
RUN apt install -y gdb gdbserver

# 1) Dépendances Qt + Wayland + xeyes pour test X11 éventuel
RUN apt-get update && apt-get install -y --no-install-recommends \
      # libs LiDAR, ROS, etc. ici… \
      qtbase5-dev qt5-qmake libqt5gui5 \
      qtwayland5 libqt5waylandclient5 libqt5waylandcompositor5 \
      x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Create ROS2 Workspace
RUN mkdir -p $HOME/ros2_ws/src

# Build workspace packages, mounting only for the build
WORKDIR $HOME/ros2_ws
RUN --mount=type=bind,destination=$HOME/ros2_ws/src/lidarSplineFilter source /opt/ros/humble/setup.bash && \
    colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-select \
    estimate_msgs \
    livox_ros_driver \
    livox_interfaces \
    livox_ros_driver2 \
    mocap4r2_msgs \
    resple

# Add workspace to default source
RUN echo "source $HOME/ros2_ws/install/setup.bash" >> $HOME/.bashrc

