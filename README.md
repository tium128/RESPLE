# RESPLE: Recursive Spline Estimation for LiDAR-Based Odometry
<div align="center">
    Video|
    Paper
</div>
<br>
    
This the offcial repository for RESPLE, the first B-spline-based recursive state estimation framework for estimating 6-DoF dynamic motions. Using RESPLE as the estimation backbone, we developed a unified suite of direct LiDAR-based odometry systems, including:
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

## Docker Build

```
cd ~/path/to/src
git clone --recursive git@github.com:ASIG-X/RESPLE.git
cd RESPLE
docker build --ssh default --tag resple .
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

* HelmDyn dataset (Livox Mid360)
```
source install/setup.bash
ros2 launch resple resple_helmdyn01.launch.py
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/bag/
```
* R-Campus dataset (Livox Avia)

```
source install/setup.bash
ros2 launch resple resple_r_campus.launch.py
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/bag/
```

* NTU VIRAL dataset (OUSTER OS1-16)
```
source install/setup.bash
ros2 launch resple resple_eee_02.launch.py
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/hesai_livox_ap20_converted.mcap
```

* MCD dataset (Livox Mid70)
```
source install/setup.bash
ros2 launch resple resple_ntu_day_01.launch.py
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/hesai_livox_ap20_converted.mcap
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

### Docker

Replacing `/path/to/data` with the location of the datasets, and `<filename>` with the launch file from above:
```
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix/ -v /path/to/data/resse_dataset/:/root/data/ resse_dataset/ --name resple resple
ros2 launch resple <filename>.launch.py
```
Note: for development, one can additionally mount the source directory with `-v .:/root/ros2_ws/src/RESPLE` and recompile with `colcon build --packages-select resple`.

In a second terminal, replacing `<example>/<filename>` to make a valid bag filepath:
```
docker exec -it resple bash
ros2 bag play ~/data/resple_dataset/<example>/
```

## License
The source code is released under [GPLv3](https://www.gnu.org/licenses/) license.
