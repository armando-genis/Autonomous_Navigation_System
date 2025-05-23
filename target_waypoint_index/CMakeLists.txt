cmake_minimum_required(VERSION 3.8)
project(target_waypoint_index)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(target_waypoint_index_node 
                src/target_waypoint_index_node.cpp # main file
              )

ament_target_dependencies(target_waypoint_index_node 
                          rclcpp 
                          geometry_msgs 
                          std_msgs 
                          visualization_msgs 
                          nav_msgs 
                          tf2 
                          tf2_ros 
                          tf2_geometry_msgs
                          Eigen3
                        )

target_include_directories(target_waypoint_index_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(target_waypoint_index_node Eigen3::Eigen)

# Install executables and libraries
install(TARGETS
  target_waypoint_index_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include/
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
