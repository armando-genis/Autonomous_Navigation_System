cmake_minimum_required(VERSION 3.8)
project(waypoints_routing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(lanelet2_core)
find_package(lanelet2_io)
find_package(lanelet2_maps)
find_package(lanelet2_projection)
find_package(lanelet2_routing)
find_package(lanelet2_traffic_rules)
find_package(lanelet2_validation)
find_package(polygon_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(waypoints_routing_node src/waypoints_routing.cpp)

ament_target_dependencies(waypoints_routing_node
  rclcpp
  visualization_msgs
  Eigen3
  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  polygon_msgs
  tf2_ros
)

target_include_directories(waypoints_routing_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install executables and libraries
install(TARGETS
  waypoints_routing_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and src files
install(
  DIRECTORY launch src config
  DESTINATION share/${PROJECT_NAME}
)

message(=============================================================)
message("-- ROS Found. ROS Support is turned ON.")
message(=============================================================)

ament_package()
