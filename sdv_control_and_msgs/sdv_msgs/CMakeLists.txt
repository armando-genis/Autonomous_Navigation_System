cmake_minimum_required(VERSION 3.8)
project(sdv_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(sdv_msgs
  "msg/PerceptionMode.msg"
  "msg/NonlinearFunctions.msg"
  "msg/State.msg"
  "msg/PanelMsg.msg"
  "msg/XboxMsg.msg"
  "msg/ThrottleMsg.msg"
  "msg/CollisionEvent.msg"
  "msg/LaneInvasionEvent.msg"
  "msg/Status.msg"
  "msg/VehicleControl.msg"
  "msg/VehicleInfo.msg"
  "msg/VehicleStatus.msg"
  "msg/VehicleInfoWheel.msg"
  "msg/GuidanceWaypoints.msg"
  "msg/EtaPose.msg"
  "msg/SystemDynamics.msg"
  "msg/ThrustControl.msg"
  "msg/Encoder.msg"
  "srv/Uint8.srv"
  DEPENDENCIES geometry_msgs std_msgs)

ament_package()