#!/usr/bin/env python3
"""display.launch.py —— 只看模型，不带任何控制。

这一层是“最省依赖”的验证路径：
    ros2 launch b601_description display.launch.py

启动的节点：
    robot_state_publisher  读 robot_description(xacro)，发布 /tf 与 /robot_description
    joint_state_publisher  发布全零 /joint_states（或用 GUI 拖动滑条）
    rviz2                  显示模型 + TF

它只需要 xacro / robot_state_publisher / rviz2，不碰 ros2_control 和 MoveIt，
所以模型改坏时先跑这个，能最快定位是 URDF 的问题还是控制层的问题。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare("b601_description")

    hardware = LaunchConfiguration("hardware")
    use_gui = LaunchConfiguration("use_joint_state_publisher_gui")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                PathJoinSubstitution([description_share, "urdf", "b601_dm.urdf.xacro"]),
                " hardware:=",
                hardware,
                " initial_positions_file:=",
                PathJoinSubstitution([description_share, "config", "initial_positions.yaml"]),
                " project_config_file:=",
                PathJoinSubstitution([description_share, "config", "b601_project.yaml"]),
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware", default_value="mock",
                                  description="mock / dm / mujoco，只影响 ros2_control 插件"),
            DeclareLaunchArgument("use_joint_state_publisher_gui", default_value="true",
                                  description="用滑条拖动关节（需要 joint_state_publisher_gui）"),
            DeclareLaunchArgument("use_rviz", default_value="true"),

            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description, "use_sim_time": False}],
            ),

            # 没有 ros2_control 时由它喂 /joint_states，让 TF 动起来。
            # 两个变体都要显式拿到 robot_description，才能知道有哪些关节。
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                condition=UnlessCondition(use_gui),
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                condition=IfCondition(use_gui),
            ),

            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", PathJoinSubstitution([description_share, "rviz", "b601_display.rviz"])],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
