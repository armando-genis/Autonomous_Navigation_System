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

sudo apt-get install libpugixml-dev
sudo apt-get install libgeographic-dev geographiclib-tools


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
#sudo apt install ros-$ROS_DISTRO-lanelet2
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

git clone https://github.com/KIT-MRT/mrt_cmake_modules.git

```

## 🏎️ Launcher
```bash
ros2 launch global_navigation_launch lidar_subprocessing.launch.py
ros2 launch robot_description display.launch.py
ros2 launch global_navigation_launch lio_sam.launch.py
```



## changes:
In the file called /lanelet2_projection/LocalCartesian.cpp i change the to this when using localcartesian map type in orden to get the ele attribute from the oms correctly and not modify. 

```bash
BasicPoint3d LocalCartesianProjector::forward(const GPSPoint& gps) const {
  BasicPoint3d local{0., 0., 0.};
  this->localCartesian_.Forward(gps.lat, gps.lon, gps.ele, local[0], local[1], local[2]);
  local[2] = gps.ele;
  return local;
}
```

also wacht that in the "lanelet2_io/io_handlers/OsmFile.cpp" this is in the code:

```bash
  static Nodes readNodes(const pugi::xml_node& osmNode) {
    Nodes nodes;
    for (auto node = osmNode.child(keyword::Node); node;  // NOLINT
         node = node.next_sibling(keyword::Node)) {
      if (isDeleted(node)) {
        continue;
      }
      const auto id = node.attribute(keyword::Id).as_llong(InvalId);
      const auto attributes = tags(node);
      const auto lat = node.attribute(keyword::Lat).as_double(0.);
      const auto lon = node.attribute(keyword::Lon).as_double(0.);
      // const auto ele = node.attribute(keyword::Elevation).as_double(0.);

      const auto ele = node.find_child_by_attribute(keyword::Tag, keyword::Key, keyword::Elevation)
                           .attribute(keyword::Value)
                           .as_double(0.);

      // std::cout << "ele: " << ele << std::endl;

      nodes[id] = Node{id, attributes, {lat, lon, ele}};
    }
    return nodes;
  }
```