cmake_minimum_required(VERSION 3.16)
project(b601_mujoco NONE)

# 纯资源包：只有 MJCF 模型，没有编译目标。
# 装成 share/b601_mujoco/models/*.xml，这样 xacro 的 $(find b601_mujoco) 与
# MujocoSystemHardware 的 mj_loadXML 都能拿到绝对路径。
find_package(ament_cmake REQUIRED)

install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

ament_package()
