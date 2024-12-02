cmake_minimum_required(VERSION 3.8)
project(pointcloud_rotation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# External dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io filters cuda)

# Link directories
link_directories(${PCL_LIBRARY_DIRS})

# Add executable
add_executable(pointcloud_rotation_node src/pointcloud_rotation_node.cpp)

# Link libraries
ament_target_dependencies(pointcloud_rotation_node
  rclcpp
  std_msgs
  sensor_msgs
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

target_include_directories(pointcloud_rotation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
  pointcloud_rotation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files (e.g., launch files)
install(
  DIRECTORY launch src config
  DESTINATION share/${PROJECT_NAME}
)

target_link_libraries(pointcloud_rotation_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

message(=============================================================)
message("-- ROS Found. ROS Support is turned ON.")
message(=============================================================)


ament_package()
