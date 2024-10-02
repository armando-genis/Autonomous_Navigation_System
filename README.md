# Autonomous_Navigation_System
 
## 📥 Install dependencies
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

#GTSAM librari for LIO-SAM
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

## 📦 Install github repository
Install the github repository inside the folder
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

```

## 🏎️ Launcher
```bash
ros2 launch global_navigation_launch lidar_subprocessing.launch.py
```

