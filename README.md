# B601-DM v0.2.0 · 主机侧 MIT

`b601_mit_arm` 在主机计算 PD 力矩，经 `effort` 和 `mit_torque` 下发。电机帧 Kp、Kd
和三项前馈为零。历史真机六轴五姿态全部通过，各姿态最大稳态误差为 0.055–0.058 rad，
归中误差 0.0210 rad。见 [验收记录](validation/host-pd.json)。

## 环境和离线门禁

在本标签的检出目录执行。需要 Ubuntu 22.04、ROS 2 Humble、ros2_control、MoveIt 2、Python
`motorbridge` 和 `mujoco`。可用 `./scripts/install_deps.sh core control moveit` 查看 ROS 依赖。

```bash
./scripts/build.sh
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
./scripts/verify.sh
./scripts/preflight.sh
```

真机测试前固定机械臂，清空扫掠范围和夹爪，确认有人可以立即急停或断电。
在归中位置开始。使能前完成探针回读、只读和方向核对。同一串口只运行一个控制程序。
常规收工先核对归中再失能。触发急停时由安全节点立即停止。

## 六轴 MIT 真机复测

先生成验收参数和回读电机。六轴使用原始设备的 MIT 固件模式。下列 launch 保持电机未使能，并会设置和核对固件模式。
方向核对后归中，退出该 launch 并撤离扫掠区。

```bash
./scripts/hw_mit_test1.sh --dry-run --amp big --slew 0.12
./scripts/dm_probe.py
ros2 launch b601_bringup bringup.launch.py hardware:=dm dm_command_mode:=mit_torque dm_enable_motors:=false \
  controllers:="joint_state_broadcaster b601_mit_arm" controllers_file:="$PWD/.verify/mit_pd_hw1.yaml"
```

现场确认后执行原始五姿态参数。脚本记录轨迹与反馈，按 MIT 回路归中并核对失能。

```bash
./scripts/hw_mit_test1.sh --execute --amp big --slew 0.12
```

历史验收范围为六个手臂轴，误差基准是各轴软限位全行程的 5%。软件位置误差会产生静态持位力矩。
历史位置回路的复测入口见 [pos_vel 流程](validation/pos-vel.md)。
