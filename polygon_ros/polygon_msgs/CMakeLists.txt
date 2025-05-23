cmake_minimum_required(VERSION 3.5)
project(polygon_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/ComplexPolygon2D.msg
    msg/ComplexPolygon2DCollection.msg
    msg/ComplexPolygon2DStamped.msg
    msg/Point2D.msg
    msg/Polygon2D.msg
    msg/Polygon2DCollection.msg
    msg/Polygon2DStamped.msg
  DEPENDENCIES
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
