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

## 统一配置

工程参数集中在 `ros2_ws/src/b601_description/config/b601_project.yaml`。该文件管理关节与电机参数、
控制和安全门槛、URDF 形状与惯量、MoveIt 规划、仿真场景、演示节点和测试工具。ROS 运行时参数和
脚本命令行选项可覆盖对应默认值。

整机运动约束由 `joints.min_pos/max_pos`、`joints.max_velocity`、`joints.max_effort` 和
`target_slew` 提供。手臂速度上限为 6.0 rad/s，夹爪为 0.05 m/s。位置目标的斜率分别为
0.08 rad/s 和 0.02 m/s。MIT 的总输出包括 PD 和前馈。关节端 MIT 在下发前用最新反馈估算
总力矩，电机仍按内部高速闭环计算。温度、总线故障和反馈失效继续触发停机保护。

```bash
./scripts/sync_config.py           # 同步生成的 ROS 参数文件和 SRDF 命名姿态
./scripts/sync_config.py --check   # 检查派生文件同步状态
```

修改模型或仿真场景后，构建工作空间并加载其环境，再生成 MuJoCo 模型。

```bash
./scripts/build.sh
source ros2_ws/install/setup.bash
model="$(mktemp)"
if python3 scripts/gen_mjcf.py > "$model"; then
  cp "$model" ros2_ws/src/b601_mujoco/models/b601_arm.xml
fi
rm "$model"
```

位置回路使用 `./scripts/dm_park.sh` 核对归中并失能，`--keep-enabled` 用于只归中核对。
关节端 MIT 验收脚本使用自身的归中和失能流程。

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

真机复测使用整机的关节速度限幅、总力矩限幅和目标斜率。
日志、CSV 和判定写入 `.verify/mit_hardware_five_poses/`。需要在试验中途停止发新目标并尝试安全归中时，
在输出目录创建 `HOLD_HARDWARE`；遇到危险立即使用现场急停。参数集中在
`ros2_ws/src/b601_description/config/b601_project.yaml`，改后执行 `./scripts/sync_config.py` 和构建。

历史真机记录可离线重算。

```bash
python3 scripts/mit_five_poses.py --score-hardware validation/results.json
```

历史位置回路的复测入口见 [pos_vel 流程](validation/pos-vel.md)。
