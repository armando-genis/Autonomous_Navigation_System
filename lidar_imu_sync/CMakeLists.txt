cmake_minimum_required(VERSION 3.8)
project(lidar_imu_sync)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
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

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


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


add_executable(lidar_imu_fusion_node src/LidarImuSync.cpp)

ament_target_dependencies(lidar_imu_fusion_node rclcpp std_msgs pcl_conversions sensor_msgs message_filters tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(lidar_imu_fusion_node Eigen3::Eigen)
target_link_libraries(lidar_imu_fusion_node ${OpenCV_LIBS})

target_link_libraries(lidar_imu_fusion_node ${PCL_LIBRARIES})

install(TARGETS
  lidar_imu_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch src
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
