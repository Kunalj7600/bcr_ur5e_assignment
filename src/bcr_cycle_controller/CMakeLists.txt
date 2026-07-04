cmake_minimum_required(VERSION 3.8)
project(bcr_cycle_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(cycle_controller src/cycle_controller.cpp)

ament_target_dependencies(cycle_controller
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
)

target_compile_features(cycle_controller PUBLIC cxx_std_17)

install(TARGETS cycle_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
