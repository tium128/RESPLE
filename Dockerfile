# ROS base
FROM osrf/ros:humble-desktop-full

# Use bash for RUN commands
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV HOME=/root

# Install system and ROS dependencies in one layer
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    openssh-client \
    libpcl-dev \
    ros-humble-pcl-conversions \
    ros-humble-pcl-msgs \
    libeigen3-dev \
    libomp-dev \
    libpcap-dev \
    x11-apps \
    qtbase5-dev \
    qt5-qmake \
    libqt5gui5 \
    qtwayland5 \
    libqt5waylandclient5 \
    libqt5waylandcompositor5 \
    ros-humble-rosbag2-storage-mcap \
    gdb \
    gdbserver \
  && rm -rf /var/lib/apt/lists/*

# Create ROS2 workspace
WORKDIR $HOME/ros2_ws
RUN mkdir -p src

# Use default branch (usually "main") for both repos
WORKDIR $HOME/ros2_ws/src
RUN git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git && \
    git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git && \
    # Clone ROS2 driver inside the SDK tree so CMake can find it
    mkdir -p rslidar_sdk/src && \
    git clone https://github.com/RoboSense-LiDAR/rs_driver.git rslidar_sdk/src/rs_driver

# Copy local RESPLE packages into workspace
# This will copy your local ROS packages into the src folder.
# Adjust the path if your packages live elsewhere.
COPY . $HOME/ros2_ws/src/

# Copy local RESPLE packages into workspace (your code including CommonUtils.h)
WORKDIR $HOME/ros2_ws/src
COPY . .

# Build the entire workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

# Expose a volume for ROS bag files
VOLUME ["/bags"]

# Source workspace on shell startup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Default entrypoint: launch RESPLE
ENTRYPOINT ["/bin/bash", "-lc", "source /opt/ros/humble/setup.bash && source $HOME/ros2_ws/install/setup.bash && ros2 launch resple resple.launch.py"]
