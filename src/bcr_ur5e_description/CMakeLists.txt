cmake_minimum_required(VERSION 3.8)
project(bcr_ur5e_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY mujoco launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
