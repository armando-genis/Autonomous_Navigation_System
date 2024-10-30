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
#sudo apt install ros-$ROS_DISTRO-lanelet2 (no longer required because in the repo is a custom repo)
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

# for the polygon represetation in rviz2 (NO LONGER REQUIERD)
# git clone https://github.com/MetroRobots/polygon_ros.git


git clone https://github.com/KIT-MRT/mrt_cmake_modules.git

```

## 📢 Code Modifications Before colcon build

Before building the package, make the following changes to the file lidar_localization_component.cpp located in the src directory of lidar_localization_ros2. These adjustments will modify the default subscriber topics to match the correct topics of the car. 
Navigate to `lidar_localization_ros2/src/lidar_localization_component.cpp` and change the lines 234 and 238 for this ones:

```bash
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_rotated", rclcpp::SensorDataQoS(),
      std::bind(&PCLLocalization::cloudReceived, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "vectornav/imu", rclcpp::SensorDataQoS(),
      std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1));
```

## 📥 Building
If it is the fist time you build the workspace follow the next commands to do not crash your computer. 
 ```bash
colcon build --packages-select lio_sam
colcon build --packages-select ndt_omp_ros2
colcon build --packages-select lidar_localization_ros2
colcon build --packages-select vectornav_msgs
colcon build --packages-select vectornav
colcon build --packages-select velodyne_msgs
colcon build --packages-select velodyne_driver
colcon build --packages-select velodyne_laserscan
colcon build --packages-select velodyne_pointcloud
colcon build --packages-select velodyne
colcon build --packages-select lidar_imu_sync
colcon build --packages-select mapping_localization_launch
colcon build --packages-select polygon_msgs
colcon build
```

## 🏅 lidar config 
`sudo ifconfig enp2s0 10.66.171.101`

## 🏅 IMU Permistion
`sudo chmod 666 /dev/ttyUSB0`

## 💡 Sensor Launchers
```bash
ros2 launch sensors_launch velodyne-VLP32C-launch.py #for lidar only
ros2 launch sensors_launch vectornav.launch.py #for IMU
ros2 launch sensors_launch lidar_imu.launch.py #for Imu and lidar 
```

## 🌏 Launchers for mapping
```bash
ros2 launch global_navigation_launch lidar_subprocessing.launch.py
ros2 launch robot_description display.launch.py
ros2 launch global_navigation_launch lio_sam.launch.py
```

## 🛰️ Launchers for Localization
```bash
ros2 launch global_navigation_launch lidar_subprocessing.launch.py
ros2 launch robot_description localization_display.launch.py
ros2 launch global_navigation_launch lidar_localization_ros2.launch.py
```

## 🛣️ Launchers for HD map & waypoints routing
```bash
ros2 launch map_visualizer osm_visualizer.launch.py
ros2 launch waypoints_routing waypoints.launch.py
```

## 🛑 changes:

### - Lanelet changes
In the file called /lanelet2_projection/LocalCartesian.cpp I change the to this when using localcartesian map type in orden to get the ele attribute from the oms correctly and not modify. 

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

### - Polygon ros changes: 

The Polygon2D structure has been modified to include a z offset, allowing for multiple polygons with different z offsets. The following files were updated to incorporate these changes:

- polygon_rviz_plugins/src/polygons_display.cpp
- polygon_rviz_plugins/src/polygon_parts.cpp
- polygon_rviz_plugins/include/polygon_rviz_plugins/polygon_base.hpp

These modifications enable the use of 3D polygons with varied z positions in the ROS environment.

Polygon2D.msg: 
```bash
# Vertices of a simple polygon. Adjacent points are connected, as are the first and last.
float64 z_offset
polygon_msgs/Point2D[] points
```

polygon_base.hpp:
```bash
  void updateProperties()
  {
    resetOutlines();
    if (mode_property_->shouldDrawOutlines())
    {
      Ogre::ColourValue outline_color = rviz_common::properties::qtToOgre(outline_color_property_->getColor());
      for (unsigned int i = 0; i < saved_outlines_.size(); ++i)
      {
        double z_offset = saved_outlines_[i].z_offset;  // Use the z_offset from each polygon
        outline_objects_[i]->setPolygon(saved_outlines_[i], outline_color, z_offset);
      }
    }

    if (!mode_property_->shouldDrawFiller() || saved_fillers_.empty())
    {
      resetFillers();
    }
    else
    {
      for (unsigned int i = 0; i < saved_fillers_.size(); ++i)
      {
        double z_offset = saved_fillers_[i].outer.z_offset;  // Use the z_offset from each complex polygon
        filler_objects_[i]->setPolygon(saved_fillers_[i], filler_colors_[i % filler_colors_.size()], z_offset);
      }
    }
  }
```


## 🛣️ Considerations for Creating HD Maps with Vector Map Builder

When creating a `Lanelet2Map` in the Vector Map Builder, follow these steps to configure the map projection:

1. Click on **Change Map Project Info**.
2. Select **Set MGRS from Lat/Lon** and input the following coordinates:
   - **Latitude:** `49`
   - **Longitude:** `8.4`
3. Click **Convert** to apply these settings.

> **Note:** When exporting the map, you may encounter an error indicating that the component `x` or `y` is negative. This error can be safely ignored, as it does not impact the map creation process. Proceed with creating the map even if these errors appear.
