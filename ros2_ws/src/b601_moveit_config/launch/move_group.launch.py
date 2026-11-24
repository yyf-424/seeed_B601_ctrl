#!/usr/bin/env python3
"""move_group.launch.py —— 只起 MoveIt，不自带机器人本体。

用途：机器人已经用别的方式跑起来了（比如 bringup 单独一个终端、或接真机调试），
只想再挂一个 MoveIt 上去规划/执行。

    # 终端 A：机器人 + 控制器
    ros2 launch b601_bringup bringup.launch.py hardware:=mock

    # 终端 B：MoveIt
    ros2 launch b601_moveit_config move_group.launch.py
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from moveit_common import build_moveit_configs, moveit_rviz_config_file, rviz_parameters  # noqa: E402


def _launch(context, *args, **kwargs):
    hardware = LaunchConfiguration("hardware").perform(context)
    # perform() 返回的是字符串，必须显式比较；直接 if 判断 "false" 也是真
    use_rviz = LaunchConfiguration("use_rviz").perform(context) == "true"
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"

    moveit_config = build_moveit_configs(hardware=hardware)

    actions = [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
        )
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
    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware", default_value="mock"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_launch),
        ]
    )
