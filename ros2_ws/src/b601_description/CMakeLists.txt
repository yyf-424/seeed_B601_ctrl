cmake_minimum_required(VERSION 3.16)
project(b601_description NONE)

# 纯资源包（xacro / yaml / rviz / launch），不含编译目标。
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
