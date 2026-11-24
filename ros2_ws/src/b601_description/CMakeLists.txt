cmake_minimum_required(VERSION 3.19)
project(b601_description NONE)

# 安装机器人模型、配置和 Python 配置读取模块。
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(project_config "${CMAKE_CURRENT_SOURCE_DIR}/config/b601_project.yaml")
set(config_generator "${CMAKE_CURRENT_SOURCE_DIR}/config/generate_cpp_config.py")
set(generated_header "${CMAKE_CURRENT_BINARY_DIR}/include/b601_description/project_parameters.hpp")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${project_config}" "${config_generator}")
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${config_generator}" "${project_config}" "${generated_header}"
  COMMAND_ERROR_IS_FATAL ANY
)
install(FILES "${generated_header}" DESTINATION include/b601_description)
ament_export_include_directories(include)

install(
  DIRECTORY urdf config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
