cmake_minimum_required(VERSION 3.16)
project(b601_moveit_config NONE)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
