#!/usr/bin/env python3
"""demo.launch.py —— 一条命令拉起完整 MoveIt 演示环境。

    ros2 launch b601_moveit_config demo.launch.py                 # mock 后端，默认
    ros2 launch b601_moveit_config demo.launch.py hardware:=dm    # 接真机

它做的事情：

    ① 包含 b601_bringup/bringup.launch.py   → robot_state_publisher + ros2_control + controller
    ② 启动 move_group                        → 规划 / IK / 碰撞检测 / 轨迹下发
    ③ 启动 rviz2 (带 MotionPlanning 面板)     → 拖末端、选目标、点 Plan & Execute

注意 ①②③ 的分工：move_group 自己不动电机，它把轨迹交给
/<controller>/follow_joint_trajectory；controller 再写 command interface；
最后由硬件插件（mock / dm / mujoco）决定“这条命令怎么变成现实”。
这条链条是本项目最值得记住的一张图（见 docs/architecture.md）。

为什么用 OpaqueFunction：
    move_group 的参数必须在“launch 真正执行的那一刻”才知道 hardware 是什么，
    普通 launch 描述阶段拿不到 LaunchConfiguration 的解析值。
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 让 launch 能 import 同目录的 moveit_common.py（MoveIt 官方示例同样这么写）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from moveit_common import build_moveit_configs, moveit_rviz_config_file, rviz_parameters  # noqa: E402


def _launch_moveit(context, *args, **kwargs):
    hardware = LaunchConfiguration("hardware").perform(context)
    # perform() 返回的是字符串，必须显式比较；直接 if 判断 "false" 也是真
    use_rviz = LaunchConfiguration("use_rviz").perform(context) == "true"
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"

    moveit_config = build_moveit_configs(hardware=hardware)

    actions = [
        # move_group：MoveIt 的主节点。参数就是上面 builder 收拢的那一堆。
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
        ),
    ]

    if use_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                output="log",
                arguments=["-d", moveit_rviz_config_file()],
                parameters=rviz_parameters(moveit_config, use_sim_time),
            )
        )

    return actions


def generate_launch_description():
    bringup_share = FindPackageShare("b601_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware", default_value="mock",
                                  description="mock（无实物）/ dm（真机）/ mujoco（动力学仿真）"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),

            # ① 机器人本体 + 控制层。这里把 rviz 关掉，因为下面要开带 MoveIt 面板的那个。
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "bringup.launch.py"])
                ),
                launch_arguments={
                    "hardware": LaunchConfiguration("hardware"),
                    "use_rviz": "false",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),

            # ②③ move_group + rviz
            OpaqueFunction(function=_launch_moveit),
        ]
    )
