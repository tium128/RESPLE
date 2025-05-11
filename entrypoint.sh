#!/usr/bin/env bash
set -e

# 1) source des environnements ROS
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash


# 2) lance foxglove_bridge en arrière-plan
ros2 launch foxglove_bridge foxglove_bridge_launch.xml __port:=8765

# 3) lance ensuite ton application Resple au premier plan
#    Remplace <TON_LAUNCH> par le nom exact (sans .launch.py) de ton launch-file
#    Exemples possibles : resple_airy96, resple_dog, resple_ntu_day_01, etc.
# exec ros2 launch resple <TON_LAUNCH>

# Exemple concret, si tu veux démarrer “resple_ntu_day_01” :
# exec ros2 launch resple resple_ntu_day_01
