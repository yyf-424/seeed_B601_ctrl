# B601-DM v0.2.1 · 关节端 MIT

主机发 `pos_target`、`vel_target`、`kp`、`kd` 和 `tff`。电机执行 PD 闭环。前馈口可接
重力、摩擦和任务力矩，验收配置三项均为零。手臂六轴和主动夹爪五姿态通过 5/5，
逐轴通过 35/35。见 [发布记录](CHANGELOG.md) 和 [逐姿态反馈](validation/results.json)。

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

## 6+1 MIT 真机复测

碰撞核对使用 Seeed 官方 `reBotArmController_ROS2` 中的 DM 模型资源，历史实测使用
`87780c6a5c2fb43b55ddebda5eac375ceb9c5a40`。把参考仓库的 `src/rebotarm_bringup` 目录
设为 `B601_VENDOR_DESCRIPTION`。缺省查找本项目同级 `rebotarm_ros2/src/rebotarm_bringup`。
只用 URDF 和网格文件，ROS 包由当前标签构建。

```bash
export B601_VENDOR_DESCRIPTION=/path/to/rebotarm_ros2/src/rebotarm_bringup
./scripts/hw_mit_five_poses.sh --simulate
./scripts/hw_mit_five_poses.sh --probe
./scripts/hw_mit_five_poses.sh --readonly
```

确认探针回读的七台电机处于 MIT 模式，方向已核对，机械臂与夹爪均处于配置的停放范围。
现场确认后执行。入口先检查厂商网格碰撞，再从小幅动作开始。

```bash
./scripts/hw_mit_five_poses.sh --execute
```

真机流程用此标签中的验收增益、0.08 rad/s 手臂目标斜率和 0.65 rad/s 独立急停阈值。
日志、CSV 和判定写入 `.verify/mit_hardware_five_poses/`。需要在试验中途停止发新目标并尝试安全归中时，
在输出目录创建 `HOLD_HARDWARE`；遇到危险立即使用现场急停。参数集中在
`ros2_ws/src/b601_description/config/b601_project.yaml`，改后执行 `./scripts/sync_config.py` 和构建。

历史真机记录可离线重算。

```bash
python3 scripts/mit_five_poses.py --score-hardware validation/results.json
```

历史位置回路的复测入口见 [pos_vel 流程](validation/pos-vel.md)。
