# B601-DM pos_vel 位置回路

ROS 2 轨迹控制器发送位置指令。硬件插件发送角度目标和速度上限，电机固件执行位置与速度闭环。
历史真机五姿态全部通过，六轴最大稳态误差 0.0047 rad。原始记录可通过 `git show v0.1.0:validation/pos-vel.json` 查阅。

## 在 MuJoCo 复测 MoveIt 2

`v1.0.1` 已分别验证仅规划和规划执行，返回 34 点与 33 点轨迹。使用两个终端，在当前工作区加载环境并设置同一个空闲 ROS 域，例如 `225`。

```bash
source scripts/ros_env.sh && b601_source_workspace
export ROS_DOMAIN_ID=225 ROS_LOCALHOST_ONLY=1
```

终端一启动 MuJoCo 位置回路与 MoveIt。

```bash
ros2 launch b601_moveit_config demo.launch.py hardware:=mujoco use_rviz:=false
```

终端二分别执行规划与规划加执行。

```bash
ros2 run b601_demos moveit_pose_goal --ros-args -p plan_only:=true
ros2 run b601_demos moveit_pose_goal --ros-args -p plan_only:=false
```

轨迹点数随规划结果变化。执行验收以 MoveIt 返回成功和 `b601_arm_controller` 报告到位为准。MIT 网页和目标仲裁通过独立入口复测。见 [上层框架验收](target-web.md)。

## 环境和离线门禁

在当前检出目录执行。需要 Ubuntu 22.04、ROS 2 Humble、ros2_control、MoveIt 2、Python
`motorbridge` 和 `mujoco`。可用 `./scripts/install_deps.sh core control moveit` 查看 ROS 依赖。

```bash
./scripts/build.sh
source scripts/ros_env.sh && b601_source_workspace
./scripts/verify.sh
./scripts/preflight.sh
```

真机测试前固定机械臂，清空扫掠范围和夹爪，确认有人可以立即急停或断电。
在归中位置开始。使能前完成探针回读、只读和方向核对。同一串口只运行一个控制程序。
常规收工先核对归中再失能。触发急停时由安全节点立即停止。

## 五姿态真机复测

先运行探针和未使能 bringup。驱动会设置并核对固件控制模式，期间可核对六轴反馈和方向，回到标定的竖直停放位，
退出只读 bringup 并撤离扫掠区。

```bash
./scripts/dm_probe.py
ros2 launch b601_bringup bringup.launch.py hardware:=dm dm_command_mode:=pos_vel dm_enable_motors:=false
```

现场确认后，终端一启动位置回路，终端二执行小幅检查、五姿态和归中。
两个终端使用同一个 ROS 工作区和 DDS 域。

```bash
# 终端一
ros2 launch b601_bringup bringup.launch.py hardware:=dm dm_command_mode:=pos_vel dm_enable_motors:=true

# 终端二
./scripts/dm_first_move.py --joint joint1 --delta 0.05
./scripts/dm_park.sh --keep-enabled
./scripts/validate_pos_vel.sh --dry-run
./scripts/validate_pos_vel.sh --execute
./scripts/dm_park.sh
```

`validate_pos_vel.sh` 从 `b601_project.yaml` 读取历史五姿态。每个姿态等轨迹完成、反馈停稳后，
按各轴软限位全行程的 5% 判定。换成 `hardware:=mock` 可用同一姿态入口检查轨迹链路。
