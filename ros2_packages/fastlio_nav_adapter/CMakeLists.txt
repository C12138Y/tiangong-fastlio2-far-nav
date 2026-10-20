cmake_minimum_required(VERSION 3.8)
project(fastlio_nav_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(
  fastlio_nav_adapter_node
  src/fastlio_nav_adapter.cpp
)

ament_target_dependencies(
  fastlio_nav_adapter_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(
  global_map_builder_node
  src/global_map_builder.cpp
)

ament_target_dependencies(
  global_map_builder_node
  rclcpp
  nav_msgs
  sensor_msgs
)

install(
  TARGETS
    fastlio_nav_adapter_node
    global_map_builder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()