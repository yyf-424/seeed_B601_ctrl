# B601-DM v0.3.0 · 关节端 MIT 变刚度

本工作区在主线提交 `1c7c19a` 上接入 [MIT 目标仲裁与网页](#mit-目标仲裁与网页)。下文真机数据来自主线验收，仲裁链路在 MuJoCo 验证。

主机下发位置、速度、增益和前馈力矩，电机执行 MIT PD 闭环。v0.3.0 的真机验收覆盖六轴高刚度跟踪、两档重力补偿、低刚度短时表现、双向切换和正常停机。MoveIt 2 属于上层规划，单独验收。见 [发布记录](CHANGELOG.md)。

## 环境和离线回归

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
真机夹爪启动位置允许在行程边界外 1 mm 内。边界处理期间 ROS 位置反馈显示相应的端点，进入行程后继续报告实测位置。
常规收工先核对归中再失能。触发急停时由安全节点立即停止。

## 统一配置

工程参数集中在 `ros2_ws/src/b601_description/config/b601_project.yaml`。该文件管理关节与电机参数、
控制和安全门槛、URDF 形状与惯量、MoveIt 规划、仿真场景、演示节点和测试工具。ROS 运行时参数和
脚本命令行选项可覆盖对应默认值。

固定增益、逐轴重力前馈及辨识模型来自 `controllers.mit`。停放位、J4 预置、分段等待、
碰撞检查默认值及归中容差来自 `recenter`。控制器 YAML 和安全配置由该文件生成。

整机运动约束由 `joints.min_pos/max_pos`、`joints.max_velocity`、`joints.max_effort` 和
`target_slew` 提供。手臂速度上限为 6.0 rad/s，夹爪为 0.05 m/s。位置目标的斜率分别为
0.5 rad/s 和 0.5 m/s。MIT 的总输出包括 PD 和前馈。关节端 MIT 在下发前用最新反馈估算
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
正常停机在实测归中通过后依次停用活动控制器、确认硬件进入 `inactive`、退出 `controller_manager`，最后回读电机状态。
位置回路和关节端 MIT 都使用 `recenter.joints` 的校准停放位。MIT 归中使用逐轴容差与整机关节限位的交集。J3 容差为 ±0.03 rad，J4 为 ±0.02 rad，其余四轴为 ±0.01 rad。

### 电机侧 MIT 归中

实机以 `hardware:=dm dm_enable_motors:=true dm_command_mode:=mit_impedance` 启动时，关节状态广播器和手臂控制器激活后，`b601_mit_recenter` 会等待实测反馈和安全状态，然后自动归中一次。使能前须确认现场可运动。上层在日志出现 `启动归中成功` 后才发送第一个目标。只读启动不执行归中。
默认归中使用关节反馈和控制配置。显式启用 `mit_recenter_check_collisions:=true` 时，应用层归中节点会请求 MoveIt 进行碰撞检查。

归中时，节点先根据反馈判断 J4。
J4 实测高于 −0.10 rad 时，先向负方向移动至少 0.10 rad，并到达 −0.10 rad 或更负；已经达到预置角时保持原角度。
随后 J1、J2、J3、J5、J6 归中，最后 J4 回落。六轴实测到位后返回成功。归中前，先让上一位目标发布者结束发令。

```bash
# 只归中并保持电机使能
./scripts/dm_park.sh --mit --keep-enabled

# 归中成功后失能
./scripts/dm_park.sh --mit

# 从其他 ROS 2 节点或命令行单独触发归中
ros2 service call /b601_mit_recenter/recenter std_srvs/srv/Trigger '{}'
```

## 电机侧 MIT

手臂只有三套现行增益，统一维护在 `b601_project.yaml` 的 `controllers.mit`。夹爪共用一套纯 PD。

| 使用阶段 | J1–J6 Kp | J1–J6 Kd |
|---|---|---|
| 无重力补偿纯 PD | 115 / 265 / 190 / 200 / 36 / 28 | 5 / 5 / 5 / 3.4 / 1.2 / 0.82 |
| 重力补偿高刚度跟踪 | 115 / 120.025 / 120.025 / 30 / 18 / 28 | 5 / 5 / 5 / 2.8 / 1.6 / 0.65 |
| 重力补偿低刚度保持 | 57.5 / 60.0125 / 60.0125 / 15 / 9 / 14 | 1 / 1 / 1 / 0.56 / 0.32 / 0.13 |

`mit_gravity_compensation` 默认关闭，使用纯 PD。开启后从高刚度起步，首个目标完成且六轴连续到位 3 秒后降档，收到新目标再升档。低档逐周期采用实测位置作为位置参考。两档共用实机辨识重力模型，高档沿用 J2–J5 的前馈比例，低档保留全轴重力比例。生成器按高档的 50% Kp 和 20% Kd 计算每轴低档增益。

### 快速启动重力补偿

在 [启动脚本](scripts/start_mit_gravity.sh) 顶部修改启动参数，保存后运行。默认使用真机电机侧 MIT、重力补偿和 `ROS_DOMAIN_ID=211`。默认会使能电机并自动归中，现场确认后再执行。只读检查时，将 `DM_ENABLE_MOTORS` 改为 `false`。

```bash
./scripts/start_mit_gravity.sh
```

保持启动终端运行。看到 `启动归中成功` 后，在第二个终端进入同一检出目录，加载工作区环境，按脚本顶部的值设置 ROS 域。下面向手臂发送一次已实测姿态，`position` 按 J1–J6 顺序填写绝对角度，单位为 rad。

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=211

python3 scripts/dm_mit_move.py --set \
  'joint1=0.36171661,joint2=-0.36209808,joint3=-0.37163483,joint4=-0.38040863,joint5=0.37049042,joint6=0.36133514'
ros2 topic echo /b601_safety/status --qos-durability transient_local
```

`limit_profile` 变为 `low_stiffness` 时开始计时。收工时先退出状态查看，再从第二个终端运行 `./scripts/dm_park.sh --mit`，完成实测归中、失能和回读。停放脚本成功后，在启动终端结束剩余节点。

### 安全档位与只读检查

驱动激活和两档急停共用 `safety.angle` 的位置边界，各端从实测端点向外扩展全行程的 5%。夹爪从动指的边界由主动指镜像。六轴低档增益到位并跟随实测位置时，急停速度为臂轴 `10 rad/s`、夹爪 `0.10 m/s`，其余时间采用常规档速度。`/b601_safety/status` 的 `limit_profile` 显示当前速度档。主动指令按 `joints` 中的保守位置限位执行。

以下命令只读取真机反馈。使能和运动需要现场确认。

```bash
ros2 launch b601_bringup bringup.launch.py hardware:=dm \
  dm_command_mode:=mit_impedance mit_gravity_compensation:=true \
  dm_enable_motors:=false use_rviz:=false
```

仿真回归使用相同高低增益，夹具的前馈系数由 MuJoCo 模型质量和质心计算。检查 5% 跟踪误差带、自动降档和低档 5 秒的关节变化。JSON、CSV 和启动日志存放在 `.verify/`。

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=214 python3 scripts/sim_mit_ros.py \
  --gravity-varstiff-switch-test --tag varstiff_switch
```

修改统一配置后运行 `./scripts/sync_config.py` 并重新构建。旧实验档的增益见 [MIT 历史试验参数](validation/mit-parameter-history.md)。

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

仿真结果只用于评估控制器稳定性。真机复测以现场确认、探针和只读回读为准。
确认探针回读的七台电机处于 MIT 模式，方向已核对，机械臂与夹爪均处于配置的停放范围。
现场确认后执行。入口先检查厂商网格碰撞，再从小幅动作开始。

```bash
./scripts/hw_mit_five_poses.sh --execute
```

真机复测使用整机的关节速度限幅、总力矩限幅和目标斜率。
日志、CSV 和判定写入 `.verify/mit_hardware_five_poses/`。需要在试验中途停止发新目标并尝试安全归中时，
在输出目录创建 `HOLD_HARDWARE`；遇到危险立即使用现场急停。参数集中在
`ros2_ws/src/b601_description/config/b601_project.yaml`，改后执行 `./scripts/sync_config.py` 和构建。

v0.2.1 的纯 PD 真机五姿态记录可离线重算。

```bash
python3 scripts/mit_five_poses.py --score-hardware validation/results.json
```

历史位置回路的复测入口见 [pos_vel 流程](validation/pos-vel.md)。
## MIT 目标仲裁与网页

MIT bringup 启动 `b601_target_arbiter` 和 `b601_target_bridge`，
根据实际加载的控制器配置建立内部路由。手动工具、分段归中、仿真与真机五姿态脚本、浏览器
和外部策略共用整机仲裁入口。同一时刻只授予一个整机会话，默认启动时没有来源持有控制权。
仲裁向各组发布 `/<控制器>/authorized_target`，转发节点是 `/<控制器>/joint_target`
的唯一正常发布者。MIT 控制器与主工作区使用同一份实现和配置。转发节点从
`/<控制器>/state` 读取主线控制器的目标序号，在 `/<控制器>/target_applied` 回报采纳编号。
标准位置模式仍使用自己的 `FollowJointTrajectory` 接口。

### 在 MuJoCo 中使用网页

```bash
./scripts/build.sh
source ros2_ws/install/setup.bash
python3 scripts/test_target_arbitration.py --web
```

浏览器打开 [本地目标控制台](http://127.0.0.1:8765/)，可以指定网页、外部策略或其他来源名。
指定 `web` 后，可用滑块或数值框编辑同一列表中的七个关节，点击「发送七关节目标」提交。
页面同时显示指定来源、当前会话持有者、实时反馈和整机回执。
三维视图使用运行中的 URDF 和关节反馈。实测模型保留 B601 的外观配色，青色模型显示
最近下发的目标或待发送预览，视图下方显示两者的末端 XYZ 坐标。拖动旋转，滚轮缩放，
「复位视角」回到初始视角。官方 STL 来自本机 `rebotarm_ros2` 参考目录，也可用
`B601_VENDOR_DESCRIPTION` 指定 `rebotarm_bringup` 目录。缺少这些网格时使用运行时 URDF 的简化外形。
宽窗口并排展示模型与关节控制，窄窗口滚动关节列表时，模型固定在屏幕上方。
在三维视图点选 TCP，按住 Ctrl 沿当前视角平面拖动。页面按 URDF 逆解末端位置并填入同一列表，
保留第七个关节的目标值。青色模型显示待发送预览，点击同一个「发送七关节目标」按钮执行。
指定外部来源后由相应客户端申请和发送目标。点击「释放控制权」清除指定来源及现有会话。
退出前释放控制权。这个入口启动本地零重力 MuJoCo 夹具，夹爪使用仿真测试增益，整机安全节点仍启用。
服务只在本机回环地址监听。常规 MIT bringup 可通过 `use_target_web:=true` 启动同一页面。

### ROS 2 接口

| 端点（前缀 `/b601_target_arbiter`） | 类型 | 用途 |
|---|---|---|
| `/set_authority` | `b601_target_interfaces/srv/SetAuthority` | 指定整机权威的来源名。切换时撤销旧会话，传空字符串清除指定并释放当前会话 |
| `/select_source` | `b601_target_interfaces/srv/SelectSource` | 以来源名申请整机会话，成功返回私有 `session_id` 和公开 `session` |
| `/release_source` | `b601_target_interfaces/srv/ReleaseSource` | 持有者释放整机会话，已指定的来源可以再次申请 |
| `/revoke_source` | `b601_target_interfaces/srv/RevokeSource` | 操作者按公开会话编号撤销指定会话 |
| `/renew_source` | `b601_target_interfaces/srv/RenewSource` | 维持现有会话，接受新目标也会续期 |
| `/set_target` | `b601_target_interfaces/srv/SetTarget` | 接收完整七关节目标，返回仲裁结果和转发编号 |
| `/source_heartbeat` | `std_msgs/msg/String` | 低开销续期，`data` 为私有 `session_id` |
| `/target` | `b601_target_interfaces/msg/TargetFrame` | 流式提交七关节目标，按会话内序号递增 |
| `/status` | `std_msgs/msg/String` | JSON，包含指定来源、当前来源、公开会话号、最近七关节目标、整机采纳编号和掉线持位状态 |

未指定来源时，第一个申请成功的客户端持有整机控制权。手动指定后，只有名称匹配的客户端可以申请。
`joint1` 到 `joint6` 的位置单位为 rad，`finger_left` 为 m。七个位置按关节名配对，也可提交七个速度。
SDK 支持局部编辑，首次用实测反馈补齐七关节，后续保留同一会话已发送的其它目标。
会话令牌用于命令和释放，状态话题只公布独立的公开编号。ROS 域内的应用应部署为可信节点，
若需隔离未经授权的参与者，由部署端设置 DDS/SROS2 发布与服务权限。

下面的调用可在上述 MuJoCo 预览的第二个终端运行。默认预览使用 `ROS_DOMAIN_ID=227`。
会话在约 0.5 秒内需要持续收到心跳或新目标，建议心跳间隔 0.1 秒。CLI 调用若分开手动输入，
间隔可能超时，下面示例从程序取得令牌并用 SDK 定时续期。相同服务也可用任意 ROS 2 客户端调用。

```bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1

python3 - <<'PYTHON'
import rclpy
from b601_demos.target_client import TargetClient

rclpy.init()
node = rclpy.create_node("external_target_example")
client = TargetClient(node, "external")
try:
    client.select(timeout=3.0)
    target = dict(zip(client.joints, [0.04, -0.3, -0.6, 0.03, 0.0, 0.0, 0.015]))
    print(client.set_joints(target, timeout=3.0))
    for _ in range(60):
        rclpy.spin_once(node, timeout_sec=0.02)
finally:
    client.close(timeout=3.0)
    node.destroy_node()
    rclpy.try_shutdown()
PYTHON

ros2 topic echo /b601_target_arbiter/status --qos-durability transient_local --once
```

流式策略用 `TargetClient(node, "policy")` 申请控制权，按策略频率调用
`client.publish(JointState(name=[...], position=[...]))`，并保持节点的 ROS executor
运行以发送定时心跳。ROS 2 原生客户端可订阅状态，调用 `select_source`，
发布带私有会话号和递增序号的 `TargetFrame`，也可用 `set_target` 取得转发编号。
`select_source` 的状态公开编号可与 `revoke_source` 一起使用。

按 `tools.target_arbitration.authority_timeout_sec=0.5`，当前来源失联或仲裁链路断流时，
转发节点把新鲜的实测位置作为普通目标送入 MIT 控制器。控制器按主线的目标斜坡和刚度策略执行。
恢复发送前需重新申请，先前会话的目标即使迟到也会被拒绝。
完成任务后主动释放时，控制器继续执行已接受的目标。关闭还持有控制权的网页会触发断流持位。
转发节点通过各组的 `target_holding` 和 `target_applied` 报告状态。断流持位依赖转发进程、
ROS 通信和新鲜反馈。使用主线自带的刚度切换时，持位目标仍可按正常策略进入低刚度。

### 复测

```bash
source ros2_ws/install/setup.bash
python3 scripts/test_target_arbitration.py
```

这个脚本会自动启动和关闭隔离 MuJoCo，验证唯一输出、并发争用、手臂与夹爪独立授权、
单关节和多关节下发、实际采纳回执、旧会话拒绝、策略流、归中交接、撤销、来源失联持位以及仲裁进程断流后的重新接管。
日志存于 `.verify/target-arbitration-227/`。完成网页预览后再运行自动验收，
或用 `B601_TEST_ROS_DOMAIN_ID` 选另一空闲 ROS 域。
