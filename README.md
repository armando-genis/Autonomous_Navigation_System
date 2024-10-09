# Autonomous_Navigation_System
 
## 📥 Install dependencies
Before installing the necessary dependencies, remember to source the appropriate ROS2 environment for your ROS2 version. This ensures the correct packages are installed for your distribution.

```bash
#fundamental libraries
sudo apt-get install libeigen3-dev
sudo apt install libpcl-dev
sudo apt-get install libpcap-dev
sudo apt install can-utils
sudo apt-get install libqt5serialport5-dev

#ros2 packages
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
sudo apt install ros-$ROS_DISTRO-xacro
sudo apt-get install ros-$ROS_DISTRO-pcl-ros
sudo apt install ros-$ROS_DISTRO-vision-msgs
sudo apt install ros-$ROS_DISTRO-perception-pcl
sudo apt install ros-$ROS_DISTRO-pcl-msgs
sudo apt install ros-$ROS_DISTRO-vision-opencv
sudo apt install ros-$ROS_DISTRO-xacro
sudo apt install ros-$ROS_DISTRO-velodyne-msgs
sudo apt install ros-$ROS_DISTRO-diagnostic-updater
sudo apt install ros-$ROS_DISTRO-lanelet2
sudo apt install ros-$ROS_DISTRO-color-util


#GTSAM librari for LIO-SAM
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

## 📦 Install github repository
Clone the GitHub repository into the `ros2_ws/src/Autonomous_Navigation_System/` folder.
```bash
# LIO SAM
git clone https://github.com/TixiaoShan/LIO-SAM.git
cd LIO-SAM/
git checkout ros2
cd ..

# ndt opm ros2
git clone https://github.com/rsasaki0109/ndt_omp_ros2

# for LIDAR localition
git clone https://github.com/rsasaki0109/lidar_localization_ros2.git

# for velodyne drivers
git clone https://github.com/ros-drivers/velodyne.git
cd velodyne
git checkout humble-devel
cd ..

# for the vectornav package
git clone https://github.com/dawonn/vectornav.git -b ros2

# for the polygon represetation in rviz2
git clone https://github.com/MetroRobots/polygon_ros.git

```

## 🏎️ Launcher
```bash
ros2 launch global_navigation_launch lidar_subprocessing.launch.py
ros2 launch robot_description display.launch.py
ros2 launch global_navigation_launch lio_sam.launch.py
```

