# ROS base
FROM osrf/ros:humble-desktop-full

# Use bash for RUN commands
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV HOME=/root

# Install system and ROS dependencies in one layer
RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential cmake \
    openssh-client \
    libpcl-dev \
    ros-humble-pcl-conversions \
    ros-humble-pcl-msgs \
    libeigen3-dev \
    libomp-dev \
    libpcap-dev \
    libqt5gui5 \
    ros-humble-rosbag2-storage-mcap \
    gdb \
    gdbserver \
    qtwayland5 \
    libqt5waylandclient5 \
    libqt5waylandcompositor5 \
    x11-apps xauth\
    libwayland-egl1-mesa \
    libwayland-client0 \
    libwayland-cursor0 \
    libxkbcommon0 \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Create ROS2 workspace
WORKDIR $HOME/ros2_ws
RUN mkdir -p src

# Use default branch (usually "main") for both repos
WORKDIR $HOME/ros2_ws/src
RUN git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git && \
    git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git && \
    #git clone https://github.com/RoboSense-LiDAR/rs_driver.git
    # Clone ROS2 driver inside the SDK tree so CMake can find it
    mkdir -p rslidar_sdk/src && \
    git clone https://github.com/RoboSense-LiDAR/rs_driver.git rslidar_sdk/src/rs_driver

# Copy local RESPLE packages into workspace
# This will copy your local ROS packages into the src folder.
# Adjust the path if your packages live elsewhere.
#COPY src $HOME/ros2_ws/src/

# 3) Copier votre workspace entier (packages + Dockerfile + bags si besoin)
WORKDIR /root/ros2_ws
COPY . .


# Si vous avez localement modifié le config/ et launch/ (plutôt que dans le dépôt),
# copiez-les ensuite :
#COPY rslidar_sdk/config /root/ros2_ws/src/rslidar_sdk/config
#COPY rslidar_sdk/launch /root/ros2_ws/src/rslidar_sdk/launch

# Copy local RESPLE packages into workspace (your code including CommonUtils.h)
#WORKDIR $HOME/ros2_ws/src
#COPY . .

# Build the entire workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --merge-install --event-handlers console_direct+
    
    # expose le port rosbridge WebSocket
    EXPOSE 8765

# Expose a volume for ROS bag files
VOLUME ["/bags"]

# 4) Passage en shell
SHELL ["/bin/bash", "-lc"]

# Copie et rend exécutable notre script de démarrage multi-process
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]