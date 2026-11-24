cmake_minimum_required(VERSION 3.16)
project(b601_hardware CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(b601_description REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ---------------------------------------------------------------------------
# 定位厂商 motor_abi 动态库
#
# 这个库随 motorbridge（官方 Python SDK 的传输层依赖）一起安装，
# 默认在 python site-packages 里，不在系统库路径中。三级查找：
#   1) CMake 变量 MOTOR_ABI_LIB 显式指定
#   2) 系统库路径（find_library）
#   3) 问 python 要 motorbridge 的安装目录
# ---------------------------------------------------------------------------
set(MOTOR_ABI_LIB "" CACHE FILEPATH "Path to libmotor_abi.so")

if(NOT MOTOR_ABI_LIB)
  find_library(MOTOR_ABI_FOUND_LIB NAMES motor_abi)
  if(MOTOR_ABI_FOUND_LIB)
    set(MOTOR_ABI_LIB "${MOTOR_ABI_FOUND_LIB}")
  endif()
endif()

if(NOT MOTOR_ABI_LIB)
  execute_process(
    COMMAND python3 -c
      "import pathlib, motorbridge; print(pathlib.Path(motorbridge.__file__).resolve().parent / 'lib')"
    OUTPUT_VARIABLE MB_LIB_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE MB_RESULT
  )
  if(MB_RESULT EQUAL 0 AND EXISTS "${MB_LIB_DIR}/libmotor_abi.so")
    set(MOTOR_ABI_LIB "${MB_LIB_DIR}/libmotor_abi.so")
    message(STATUS "b601_hardware: 从 motorbridge 找到 libmotor_abi.so -> ${MOTOR_ABI_LIB}")
  endif()
endif()

if(NOT MOTOR_ABI_LIB OR NOT EXISTS "${MOTOR_ABI_LIB}")
  message(FATAL_ERROR
    "找不到 libmotor_abi.so。三种修法：\n"
    "  1) pip install motorbridge    （推荐，之后重新 colcon build）\n"
    "  2) colcon build --cmake-args -DMOTOR_ABI_LIB=/绝对路径/libmotor_abi.so\n"
    "  3) export MOTOR_ABI_LIB=... 后重新构建")
endif()
get_filename_component(MOTOR_ABI_DIR "${MOTOR_ABI_LIB}" DIRECTORY)

# ---------------------------------------------------------------------------
# 定位 MuJoCo（仿真后端用）
#
# mujoco 是用 pip 装在用户目录的（site-packages/mujoco/{include,libmujoco.so.3.x}），
# 既没有系统头文件也没有 libmujoco.so 符号链接。三级查找：
#   1) CMake 变量 MUJOCO_ROOT 显式指定
#   2) python -c "import mujoco" 拿包目录
#   3) 常见路径 HINTS
# 找不到就把 mujoco 插件编成"空壳"（仍可编译，只是 hardware:=mujoco 会报错），
# 这样没有 mujoco 的机器也能编过其余部分。
# ---------------------------------------------------------------------------
set(MUJOCO_ROOT "" CACHE PATH "Path to mujoco package dir (含 include/ 与 libmujoco.so*)")

if(NOT MUJOCO_ROOT)
  execute_process(
    COMMAND python3 -c "import mujoco, pathlib; print(pathlib.Path(mujoco.__file__).resolve().parent)"
    OUTPUT_VARIABLE MJ_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE MJ_RESULT
  )
  if(MJ_RESULT EQUAL 0 AND EXISTS "${MJ_DIR}/include/mujoco/mujoco.h")
    set(MUJOCO_ROOT "${MJ_DIR}")
    message(STATUS "b601_hardware: 从 python 找到 mujoco -> ${MUJOCO_ROOT}")
  endif()
endif()

set(HAVE_MUJOCO 0)
if(MUJOCO_ROOT AND EXISTS "${MUJOCO_ROOT}/include/mujoco/mujoco.h")
  file(GLOB MJ_LIBS "${MUJOCO_ROOT}/libmujoco.so*")
  if(MJ_LIBS)
    list(GET MJ_LIBS 0 MUJOCO_LIB)
    set(HAVE_MUJOCO 1)
    message(STATUS "b601_hardware: mujoco 头文件与库 -> ${MUJOCO_LIB}")
  endif()
endif()

# ---------------------------------------------------------------------------
# 插件库
# ---------------------------------------------------------------------------
add_library(b601_dm_hardware SHARED
  src/dm_system.cpp
)
target_include_directories(b601_dm_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${b601_description_INCLUDE_DIRS}
)
target_link_libraries(b601_dm_hardware PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${MOTOR_ABI_LIB}
)

# 运行期依赖 RPATH：插件被 controller_manager dlopen 时，要能自己找到 libmotor_abi.so。
# 用 $ORIGIN 相对路径 + 绝对路径双保险（库在用户目录下，不进系统路径）。
set_target_properties(b601_dm_hardware PROPERTIES
  BUILD_RPATH "${MOTOR_ABI_DIR}"
  INSTALL_RPATH "${MOTOR_ABI_DIR}"
)

# ---------------------------------------------------------------------------
# MuJoCo 仿真后端插件（可选：只有找到 mujoco 才编）
# ---------------------------------------------------------------------------
if(HAVE_MUJOCO)
  add_library(b601_mujoco_hardware SHARED
    src/mujoco_system.cpp
  )
  target_include_directories(b601_mujoco_hardware PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${b601_description_INCLUDE_DIRS}
    "${MUJOCO_ROOT}/include"
  )
  target_link_libraries(b601_mujoco_hardware PUBLIC
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${MUJOCO_LIB}
  )
  get_filename_component(MUJOCO_LIB_DIR "${MUJOCO_LIB}" DIRECTORY)
  set_target_properties(b601_mujoco_hardware PROPERTIES
    BUILD_RPATH "${MUJOCO_LIB_DIR}"
    INSTALL_RPATH "${MUJOCO_LIB_DIR}"
  )
  install(TARGETS b601_mujoco_hardware
    EXPORT export_b601_hardware
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_export_libraries(b601_mujoco_hardware)
  message(STATUS "b601_hardware: 启用 MuJoCo 仿真后端插件")
else()
  message(WARNING "b601_hardware: 未找到 mujoco（pip install --user mujoco），跳过 MujocoSystemHardware")
endif()

# ---------------------------------------------------------------------------
# 安装与 pluginlib 导出
# ---------------------------------------------------------------------------
install(TARGETS b601_dm_hardware
  EXPORT export_b601_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES b601_hardware_plugins.xml DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(hardware_interface b601_hardware_plugins.xml)

ament_export_include_directories(include)
ament_export_dependencies(b601_description)
ament_export_libraries(b601_dm_hardware)
ament_export_targets(export_b601_hardware)
ament_package()
