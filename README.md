# RESPLE: Recursive Spline Estimation for LiDAR-Based Odometry
<div align="center">
    Video|
    Paper
</div>
<br>
    
This is the offcial repository for RESPLE, the first B-spline-based recursive state estimation framework for estimating 6-DoF dynamic motions. Using RESPLE as the estimation backbone, we developed a unified suite of direct LiDAR-based odometry systems, including:
* LiDAR-only odometry (LO)
* LiDAR-inertial odometry (LIO)
* Multi-LiDAR odometry (MLO)
* Multi-LiDAR-inertial Odometry (MLIO)

These four variants have been tested in real-world datasets and our own experiments, covering aerial, wheeled, legged, and wearable platforms operating in indoor, urban, wild environments with diverse LiDAR types. We look forward to your comments and feedabck! 
 
### Dependencies
Tested with [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) on Ubuntu 22.04
```
sudo apt install libomp-dev libpcl-dev libeigen3-dev
sudo apt install ros-humble-pcl*
sudo apt install ros-humble-rosbag2-storage-mcap
```
Optional: [rosbag2_storage_mcap](https://docs.ros.org/en/humble/p/rosbag2_storage_mcap/#) (for playing .mcap file if testing GrandTour dataset)


### Compilation
```
cd ~/ros2_ws/src
git clone --recursive git@github.com:ASIG-X/RESPLE.git
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select estimate_msgs livox_ros_driver livox_interfaces livox_ros_driver2 mocap4r2_msgs resple
```

## Own experimental datasets ([LINK to SURFdrive](https://surfdrive.surf.nl/files/index.php/s/lfXfApqVXTLIS9l)) 
Password: RESPLE2025

**HelmDyn (Helm Dynamic) dataset**
* 1 Livox Mid360 mounted on a helmet as a mobile platform
* 10 sequences recorded with very dynamic motions combining walking, running, jumping, and in-hand waving within a cubic space   
* Ground truth trajectoryrecorded using a high-precision (submillimeter), low-latency motion capture system (Qualisys) invovling 20 cameras

**R-Campus dataset**
* 1 Livox Avia mounted on a bipedal wheeled robot (Direct Drive DIABLO)
* 1 sequence in walking speed recorded in a large-scale campus environment
* Trajectory starts and ends at the same location point. 


## Usage
Parameters are explained in `resple/config/config_rcampus.yaml`. For LIO setting, change `if_lidar_only` in `resple/config/config_xxx.yaml` to `false`. 

* [HelmDyn](https://surfdrive.surf.nl/files/index.php/s/lfXfApqVXTLIS9l) dataset (Livox Mid360)
```
source install/setup.bash
ros2 launch resple resple_helmdyn01.launch.py
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/bag/
```
* [R-Campus](https://surfdrive.surf.nl/files/index.php/s/lfXfApqVXTLIS9l) dataset (Livox Avia)

```
source install/setup.bash
ros2 launch resple resple_r_campus.launch.py
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/bag/
```

* [NTU VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/) dataset (OUSTER OS1-16)
```
source install/setup.bash
ros2 launch resple resple_eee_02.launch.py
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/bag/
```

* [MCD](https://mcdviral.github.io/) dataset (Livox Mid70)
```
source install/setup.bash
ros2 launch resple resple_ntu_day_01.launch.py
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/bag/
```
  
* GrandTour (Hesai XT32, Livox Mid360)
```
source install/setup.bash
ros2 launch resple resple_heap_testsite_hoenggerberg.launch.py
# ros2 launch resple resple_jungfraujoch_tunnel_small.launch.py
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/hesai_livox_ap20_converted.mcap
```

## License
The source code is released under [GPLv3](https://www.gnu.org/licenses/) license.
