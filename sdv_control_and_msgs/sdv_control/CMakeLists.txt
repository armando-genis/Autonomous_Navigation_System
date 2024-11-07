cmake_minimum_required(VERSION 3.5)
project(sdv_control)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sdv_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(vectornav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(DEPENDENCIES
    sdv_msgs
    std_msgs
    rclcpp
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2
    Eigen3
    diagnostic_msgs
    vectornav_msgs
    tf2_geometry_msgs
)

set(LIBRARIES
    VANTTEC_CONTROLLERS
    # sdc1_simulation_node
    # car_tf2_broadcast_node
    # sdc_control_node
)

# TODO  Vanttec_controller should have it's own cmakelists
set(VANTTEC_CONTROLLER_SOURCES
    # DYNAMIC MODEL
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/dynamic_models/ground_vehicles/car_like/vehicles/vtec_sdc1.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/dynamic_models/ground_vehicles/car_like/base/car_3dof_dynamic_model.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/utils/utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/simulation/car_tfs/sdc1_broadcaster.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/simulation/base/tf2_6dof_broadcaster_ros2.cpp

    # PID
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/controllers/feedback_linearization/model_based_controllers/SDCs/regulators/vtec_sdc1_pid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/controllers/feedback_linearization/PID_based/first_order/fblin_pid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/controllers/feedback_linearization/base/fb_lin_control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/controllers/control_laws/PID/first_order/pid.cpp

    # ASMC
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/controllers/feedback_linearization/model_based_controllers/SDCs/regulators/vtec_sdc1_asmc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/controllers/feedback_linearization/SMC_based/ASMC/first_order/fblin_asmc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/controllers/feedback_linearization/base/fb_lin_control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/controllers/control_laws/SMC/ASMC/first_order/asmc.cpp

    # AITSMC
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/controllers/feedback_linearization/model_based_controllers/SDCs/regulators/vtec_sdc1_aitsmc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/controllers/feedback_linearization/SMC_based/AITSMC/first_order/fblin_aitsmc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/controllers/feedback_linearization/base/fb_lin_control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/controllers/control_laws/SMC/AITSMC/first_order/aitsmc.cpp

    # STANLEY CONTROLLER
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/controllers/guidance_laws/stanley_controller.cpp

    # LOS CONTROLLER
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/controllers/guidance_laws/LOS.cpp

)

set(VANTTEC_CONTROLLER_PATHS
    # ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/dynamic_models/ground_vehicles/car_like/
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/simulation/
    # ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/controllers/control_laws/PID/first_order/
    # ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/controllers/guidance_laws/
    # ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/controllers/
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/
)

#Add cpp library files
add_library(VANTTEC_CONTROLLERS SHARED
    ${VANTTEC_CONTROLLER_SOURCES}
    # ${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/dynamic_models/ground_vehicles/car_like/vehicles/vtec_sdc1.cpp
)

# Specify the include directories for the libraries
target_include_directories(VANTTEC_CONTROLLERS
  PUBLIC
  $<BUILD_INTERFACE:${VANTTEC_CONTROLLER_PATHS}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_controllers/dynamic_models/ground_vehicles/car_like/>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(VANTTEC_CONTROLLERS Eigen3::Eigen)

ament_target_dependencies(VANTTEC_CONTROLLERS ${DEPENDENCIES})

#target_compile_definitions(sdv_control PRIVATE "sdv_control_BUILDING_LIBRARY")

#   ********************** SDC1 SIMULATION NODE **********************
#Add executables
add_executable(sdc1_simulation_node
              src/model/sdc1_simulation_node.cpp)
# Link against required libraries
target_link_libraries(sdc1_simulation_node
    VANTTEC_CONTROLLERS
    # ${rclcpp_LIBRARIES}
    # ${std_msgs_LIBRARIES}
    # ${sdv_msgs_LIBRARIES}
    # ${diagnostic_msgs_LIBRARIES}
)
# Include directories for project headers and other package headers
target_include_directories(sdc1_simulation_node
            PUBLIC
            $<BUILD_INTERFACE:${VANTTEC_CONTROLLER_PATHS}>
            $<BUILD_INTERFACE:${sdv_msgs_INCLUDE_DIRS}>
            $<INSTALL_INTERFACE:include>
)
target_compile_features(sdc1_simulation_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#   ********************** TF2 BROADCAST NODE **********************
#Add executables
add_executable(car_tf2_broadcast_node
              src/model/car_tf2_broadcast_node.cpp)
# Link against required libraries
target_link_libraries(car_tf2_broadcast_node
    VANTTEC_CONTROLLERS
    # ${rclcpp_LIBRARIES}
    # ${geometry_msgs_LIBRARIES}
    # ${nav_msgs_LIBRARIES}
    # ${sdv_msgs_LIBRARIES}
)
# Include directories for project headers and other package headers
target_include_directories(car_tf2_broadcast_node
            PUBLIC
            # $<BUILD_INTERFACE:${VANTTEC_CONTROLLER_PATHS}>
            $<BUILD_INTERFACE:${sdv_msgs_INCLUDE_DIRS}>
            $<INSTALL_INTERFACE:include>
)
target_compile_features(car_tf2_broadcast_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#   ********************** SDC1 PID NODE **********************
#Add executables
add_executable(sdc1_vel_pid_node
              src/control/regulators/first_order/sdc1_vel_pid_node.cpp)
# Link against required libraries
target_link_libraries(sdc1_vel_pid_node
    VANTTEC_CONTROLLERS
    # ${rclcpp_LIBRARIES}
    # ${std_msgs_LIBRARIES}
    # ${sdv_msgs_LIBRARIES}
    # ${diagnostic_msgs_LIBRARIES}
)
# Include directories for project headers and other package headers
target_include_directories(sdc1_vel_pid_node
            PUBLIC
            $<BUILD_INTERFACE:${VANTTEC_CONTROLLER_PATHS}>
            $<BUILD_INTERFACE:${sdv_msgs_INCLUDE_DIRS}>
            $<INSTALL_INTERFACE:include>
)
target_compile_features(sdc1_vel_pid_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#   ********************** SDC1 ASMC NODE **********************
#Add executables
add_executable(sdc1_vel_asmc_node
              src/control/regulators/first_order/sdc1_vel_asmc_node.cpp)
# Link against required libraries
target_link_libraries(sdc1_vel_asmc_node
    VANTTEC_CONTROLLERS
    # ${rclcpp_LIBRARIES}
    # ${std_msgs_LIBRARIES}
    # ${sdv_msgs_LIBRARIES}
    # ${diagnostic_msgs_LIBRARIES}
)
# Include directories for project headers and other package headers
target_include_directories(sdc1_vel_asmc_node
            PUBLIC
            $<BUILD_INTERFACE:${VANTTEC_CONTROLLER_PATHS}>
            $<BUILD_INTERFACE:${sdv_msgs_INCLUDE_DIRS}>
            $<INSTALL_INTERFACE:include>
)
target_compile_features(sdc1_vel_asmc_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#   ********************** SDC1 AITSMC NODE **********************
#Add executables
add_executable(sdc1_vel_aitsmc_node
              src/control/regulators/first_order/sdc1_vel_aitsmc_node.cpp)
# Link against required libraries
target_link_libraries(sdc1_vel_aitsmc_node
    VANTTEC_CONTROLLERS
)
# Include directories for project headers and other package headers
target_include_directories(sdc1_vel_aitsmc_node
            PUBLIC
            $<BUILD_INTERFACE:${VANTTEC_CONTROLLER_PATHS}>
            $<BUILD_INTERFACE:${sdv_msgs_INCLUDE_DIRS}>
            $<INSTALL_INTERFACE:include>
)
target_compile_features(sdc1_vel_aitsmc_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#   ********************** SDC1 STANLEY NODE **********************
#Add executables
add_executable(stanley_controller_node
              src/control/guidance_laws/stanley_controller_node.cpp)
# Link against required libraries
target_link_libraries(stanley_controller_node
    VANTTEC_CONTROLLERS
    # ${rclcpp_LIBRARIES}
    # ${std_msgs_LIBRARIES}
    # ${sdv_msgs_LIBRARIES}
    # ${diagnostic_msgs_LIBRARIES}
)
# Include directories for project headers and other package headers
target_include_directories(stanley_controller_node
            PUBLIC
            $<BUILD_INTERFACE:${VANTTEC_CONTROLLER_PATHS}>
            $<BUILD_INTERFACE:${sdv_msgs_INCLUDE_DIRS}>
            $<INSTALL_INTERFACE:include>
)
target_compile_features(stanley_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#   ********************** SDC1 LOS NODE **********************
#Add executables
add_executable(los_controller_node
              src/control/guidance_laws/los_controller_node.cpp)
# Link against required libraries
target_link_libraries(los_controller_node
    VANTTEC_CONTROLLERS
    # ${rclcpp_LIBRARIES}
    # ${std_msgs_LIBRARIES}
    # ${sdv_msgs_LIBRARIES}
    # ${diagnostic_msgs_LIBRARIES}
)
# Include directories for project headers and other package headers
target_include_directories(los_controller_node
            PUBLIC
            $<BUILD_INTERFACE:${VANTTEC_CONTROLLER_PATHS}>
            $<BUILD_INTERFACE:${sdv_msgs_INCLUDE_DIRS}>
            $<INSTALL_INTERFACE:include>
)
target_compile_features(los_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


#### Install
install(TARGETS
  ${LIBRARIES}
  sdc1_simulation_node
  car_tf2_broadcast_node
  sdc1_vel_pid_node
  sdc1_vel_asmc_node
  sdc1_vel_aitsmc_node
  stanley_controller_node
  los_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY ${VANTTEC_CONTROLLER_PATHS}
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/waypoint_handler.py
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
