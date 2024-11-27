cmake_minimum_required(VERSION 3.8)
project(rviz_overlay_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

# Enable automatic MOC for Q_OBJECT macro
set(CMAKE_AUTOMOC ON)

# Add the executable or shared library
add_library(${PROJECT_NAME} SHARED src/overlay_display.cpp)

# Add include directories for RViz, OGRE, and OGRE Overlay explicitly
target_include_directories(${PROJECT_NAME} PRIVATE
  ${rviz_common_INCLUDE_DIRS}
  ${rviz_rendering_INCLUDE_DIRS}
  ${rviz_ogre_vendor_INCLUDE_DIRS}
  /opt/ros/humble/opt/rviz_ogre_vendor/include  # Root include directory
  /opt/ros/humble/opt/rviz_ogre_vendor/include/OGRE/Overlay  # Overlay directory explicitly
)

# Link the required libraries
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  ${rviz_common_LIBRARIES}
  ${rviz_rendering_LIBRARIES}
  ${rviz_ogre_vendor_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rviz_common
  rviz_rendering
  rclcpp
  Qt5Widgets
  rviz_ogre_vendor
  pluginlib)

# Export the plugin description file
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

# Install the plugin and XML file
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME})

install(FILES plugins/plugin_description.xml
  DESTINATION share/${PROJECT_NAME}/plugins)

# Call ament_package once at the end, after all configurations
ament_export_libraries(${PROJECT_NAME})
ament_package()
