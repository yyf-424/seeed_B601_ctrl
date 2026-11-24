#!/usr/bin/env python3
"""sim_mit_ros.py —— 在 MuJoCo 后端上端到端验证 MIT 力控控制器（ROS 全链路）

为什么要有它：
    `scripts/sim_mit_check.py` / `sim_mit_poses.py` 都是**纯 MuJoCo + Python 重写控制律**，
    验证的是"控制律的数学"；本脚本走真正的 ROS 链路（controller_manager → 控制器 →
    MuJoCo 硬件插件 → 安全节点），验证的是"控制器实现"。改控制器代码时必须用它做回归。

它做什么：
    1) 起 `bringup.launch.py hardware:=mujoco dm_command_mode:=mit_torque`（可指定控制器 yaml）；
    2) 等控制器 active、等到 /joint_states；
    3) 按 `--slew` 逐姿态发**斜坡**目标（不阶跃：阶跃会把手臂打飞，见 docs/test-log.md 第七轮）；
    4) 每个姿态停稳后采样稳态误差，并统计全程每关节峰值 |q̇|；
    5) 打印机器可读的汇总行 + 写一份 CSV。

用法（必须在已经 source 过的 shell 里跑，DDS 才能对上）：
    source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
    ./scripts/sim_mit_ros.py                          # 默认 3 姿态
    ./scripts/sim_mit_ros.py --controller b601_mit_controller \\
        --controllers-file ros2_ws/src/b601_bringup/config/b601_mit_controllers.yaml

退出码：0 = 全部姿态在判据内；1 = 有姿态超差或触发急停。
"""

from __future__ import annotations

import argparse
import csv
import re
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration as MsgDuration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

# 仿真专用起始位形：真值的 startup 是"竖直位"（真机上电姿态），那是**不稳定平衡点** ——
# 仿真里重力前馈稍有偏差就会倾倒，一开局就把手臂甩到 6 rad/s。所以回归用弯曲位形。
SIM_INITIAL = [0.0, -0.60, -0.60, 0.0, 0.0, 0.0]

DEFAULT_POSES = [
    [0.0, -0.45, -0.45, 0.0, 0.0, 0.0],
    [0.25, -0.65, -0.55, 0.15, 0.10, 0.25],
    [0.0, -0.80, -0.70, -0.15, -0.10, -0.25],
]


class Harness(Node):
    def __init__(self, controller: str, arm_joints: list[str]):
        super().__init__("sim_mit_ros_harness")
        self.arm_joints = arm_joints
        self.latest: dict[str, float] = {}
        self.latest_vel: dict[str, float] = {}
        self.stamp = 0.0
        self.samples: list[tuple[float, list[float], list[float]]] = []
        self.status_level = -1
        self.status_text = ""
        self.create_subscription(JointState, "/joint_states", self._on_state, 50)
        from std_msgs.msg import Float64MultiArray
        self.last_debug: list[float] = []
        self.first_debug: list[float] = []
        self.first_debug_q: list[float] = []
        self.state = n_state = []          # 最近一帧 ~/state
        self.state_log: list[tuple[float, list[float]]] = []
        self.create_subscription(
            Float64MultiArray, f"/{controller}/debug", self._on_debug, 10)
        self.create_subscription(
            Float64MultiArray, f"/{controller}/state", self._on_state_msg, 10)
        self.pub = self.create_publisher(JointState, f"/{controller}/joint_target", 10)
        self.extra_pubs = []      # 拆分实例时，其余实例的目标话题
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            # 安全状态是锁存的（transient_local），普通 volatile 订阅收不到旧样本
            __import__("diagnostic_msgs.msg", fromlist=["DiagnosticStatus"]).DiagnosticStatus,
            "/b601_safety/status",
            self._on_status,
            qos,
        )

    # ------------------------------------------------------------------ 回调
    def arm_action_client(self, controller: str) -> ActionClient:
        return ActionClient(self, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory")

    def _on_state(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self.latest[name] = pos
        for name, vel in zip(msg.name, msg.velocity):
            self.latest_vel[name] = vel
        if self.arm_joints and all(j in self.latest for j in self.arm_joints):
            self.samples.append(
                (
                    time.monotonic(),
                    [self.latest[j] for j in self.arm_joints],
                    [self.latest_vel.get(j, 0.0) for j in self.arm_joints],
                )
            )

    def _on_state_msg(self, msg) -> None:
        self.state = list(msg.data)
        self.state_log.append((time.monotonic(), list(msg.data)))

    def _on_debug(self, msg) -> None:
        if not self.first_debug:
            self.first_debug = list(msg.data)
            self.first_debug_q = self.q()[:] if self.arm_joints else []
        self.last_debug = list(msg.data)

    def _on_status(self, msg) -> None:
        level = msg.level
        if isinstance(level, (bytes, bytearray)):
            level = level[0] if level else 0
        self.status_level = int(level)
        self.status_text = msg.message

    # ------------------------------------------------------------------ 工具
    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ready(self, timeout: float) -> bool:
        """等到 /joint_states 有本实例全部关节。"""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.arm_joints and all(j in self.latest for j in self.arm_joints):
                return True
        return False

    def q(self) -> list[float]:
        return [self.latest[j] for j in self.arm_joints]

    def send(self, target: list[float]) -> None:
        msg = JointState()
        msg.name = list(self.arm_joints)
        msg.position = list(target)
        self.pub.publish(msg)
        for extra in self.extra_pubs:
            extra.publish(msg)

    def send_ramp_trajectory(self, target: list[float], slew: float, action_client,
                             result_timeout: float = 30.0) -> bool:
        """位置模式：把"当前位形 → target"拼成一条线性斜坡轨迹，走 JTC action 下发。"""
        start = self.q()
        delta = [t - s for t, s in zip(target, start)]
        duration = max(max(abs(d) for d in delta) / max(slew, 1e-3), 0.5)
        steps = max(2, int(duration / 0.1))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.arm_joints)
        for k in range(steps + 1):
            alpha = k / steps
            point = JointTrajectoryPoint()
            sec = duration * alpha
            point.positions = [s + d * alpha for s, d in zip(start, delta)]
            point.velocities = [d / duration for d in delta]
            point.time_from_start = MsgDuration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
            goal.trajectory.points.append(point)
        if not action_client.wait_for_server(timeout_sec=10.0):
            print("   ❌ 找不到 JTC action server（位置模式控制器在跑吗）")
            return False
        send_future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            print("   ❌ 轨迹 goal 被拒绝")
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=result_timeout)
        res = result_future.result()
        code = res.result.error_code if res is not None else 999
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            print(f"   ⚠ 轨迹结果 error_code={code}（0=SUCCESSFUL）")
            return code == 0
        return True

    def ramp_to(self, target: list[float], slew: float, rate_hz: float = 50.0, tag: str = "") -> None:
        """从当前位置按 slew（关节单位/s）斜坡逼近目标，绝不阶跃。"""
        start = self.q()
        delta = [t - s for t, s in zip(target, start)]
        duration = max(abs(d) for d in delta) / max(slew, 1e-3)
        steps = max(1, int(math.ceil(duration * rate_hz)))
        for k in range(1, steps + 1):
            alpha = k / steps
            self.send([s + d * alpha for s, d in zip(start, delta)])
            self.spin_for(1.0 / rate_hz)
        if tag:
            print(f"    [{tag}] 斜坡 {duration:.2f}s 完成")


def state_machine_test(node, joints, args) -> int:
    """验证「指令层 + 整组 done + 变刚度」的状态机本身（不看轨迹，看 kp/done 的时序）。

    步骤：
      A 等 1 s：应是等待档（done=1，kp≈kp_free）
      B 发一个新目标（+0.2 rad）：done→0，kp 线性升到 kp_track；到位后 3 s 内 done→1，kp 落回 kp_free
      C 在 done=1 时发 +0.0005 rad（< 死区 0.001）：**不许**重新索权（done 仍为 1、kp 不升）
      D 再发 +0.002 rad（> 死区）：必须重新索权（done→0、kp 升起）
    """
    n = len(joints)
    def snapshot():
        if not node.state or len(node.state) < 8 * n + 4:
            return None
        st = node.state
        return {
            "done": st[8 * n + 0] > 0.5,
            "seq": int(st[8 * n + 1]),
            "t_in_band": st[8 * n + 2],
            "kp": st[2 * n:3 * n],
            "band": st[6 * n:7 * n],
            "err_cmd": st[4 * n:5 * n],
        }

    def wait_state(pred, timeout, label):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            node.spin_for(0.02)
            s = snapshot()
            if s is not None and pred(s):
                return s
        print(f"   ❌ {label} 超时（{timeout:.1f}s）")
        return None

    ok = True
    node.spin_for(1.0)
    s = snapshot()
    if s is None:
        print("❌ 收不到 ~/state（控制器没在发？）")
        return 1
    print(f"  A 初始：done={s['done']} kp={json.dumps([round(k, 2) for k in s['kp']])}")

    base = node.q()
    target = list(base)
    target[0] += 0.2
    print(f"  B 发目标 joint1+0.2 rad …")
    t0 = time.monotonic()
    node.send(target)
    s = wait_state(lambda s: (not s["done"]), 2.0, "done→0（重新索权）")
    if s is None:
        ok = False
    else:
        print(f"     done=0 用时 {time.monotonic() - t0:.3f}s；band={s['band'][0]:.4f}")
    s = wait_state(lambda s: s["kp"][0] > 0.8 * 60.0, 3.0, "kp 升到 80%")
    if s is None:
        ok = False
    else:
        print(f"     kp 升到 48 用时 {time.monotonic() - t0:.3f}s（期望≈{0.8 * 0.8:.2f}s 量级）")
    s = wait_state(lambda s: s["done"], 12.0, "done→1（到位锁存）")
    if s is None:
        ok = False
    else:
        print(f"     整组 done 用时 {time.monotonic() - t0:.2f}s；此时 err={json.dumps([round(e, 4) for e in s['err_cmd']])}")
    s = wait_state(lambda s: s["kp"][0] < 1.5, 6.0, "kp 落回 kp_free")
    if s is None:
        ok = False
    else:
        print(f"     kp 回落到 {s['kp'][0]:.2f} 用时 {time.monotonic() - t0:.2f}s（含 3 s 到位判定）")

    # B2：误差带上限（大位移时 5% 会被 cap 到 settle_band_max=0.05）
    big = list(node.q())
    big[0] += 2.0                       # joint1 走 2 rad（限位 ±2.6）：5% = 0.1 > cap 0.05
    node.send(big)
    s = wait_state(lambda s: (not s["done"]), 2.0, "B2 索权")
    if s is not None:
        cap_ok = abs(s["band"][0] - 0.05) < 1e-6
        print(f"  B2 大位移 2 rad → band={s['band'][0]:.4f}（cap 0.05）"
              f"{'✅' if cap_ok else '❌'}")
        ok = ok and cap_ok
    s = wait_state(lambda s: s["done"], 12.0, "B2 到位")
    if s is None:
        ok = False

    # B3：整组 done 由**最慢**关节决定（joint1 只动 0.05、joint3 动 0.6）
    base3 = node.q()
    mix = list(base3)
    mix[0] += 0.05
    mix[2] += 0.60
    t3 = time.monotonic()
    node.send(mix)
    if wait_state(lambda s: (not s["done"]), 2.0, "B3 索权") is None:
        ok = False
    s = wait_state(lambda s: s["done"], 14.0, "B3 整组 done")
    if s is None:
        ok = False
    else:
        el = time.monotonic() - t3
        # 慢的 joint3：0.6/0.5 = 1.2 s 斜坡 + 3 s 判定 ≈ 4.2 s；快的 joint1 只需 0.1+3 = 3.1 s
        slow_ok = el > 3.6
        print(f"  B3 混合目标（j1+0.05 / j3+0.6）→ 整组 done 用时 {el:.2f}s "
              f"（最慢关节理论≈4.2s，最快≈3.1s）{'✅ 由慢关节决定' if slow_ok else '❌ 偏快'}")
        ok = ok and slow_ok

    # 回到"已到位"状态，才能验死区（否则 C 的前提不成立）
    hold = list(node.q())
    node.send(hold)
    if wait_state(lambda s: s["done"], 14.0, "回到 done") is None:
        ok = False

    # C：小于死区的目标变化不应重新索权
    s_now = snapshot()
    tiny = [node.q()[i] + (s_now["err_cmd"][i] if s_now else 0.0) for i in range(n)]
    tiny[0] += 0.0005
    seq_before = snapshot()["seq"] if snapshot() else -1
    node.send(tiny)
    node.spin_for(1.0)
    s = snapshot()
    # 判据用 done：seq 是"收到过几帧"（每帧都 +1，与是否重新索权无关），
    #   kp 可能还在从上一段 TRACK 回落；**只有 done 反映"是否重新索权"**。
    claimed = not s["done"]
    print(f"  C 发 +0.0005 rad（< 死区 0.001）：done={s['done']} seq={s['seq']}（前 {seq_before}）"
          f"→ {'❌ 不该重新索权' if claimed else '✅ 正确忽略'}")
    ok = ok and not claimed

    # D：超过死区的目标变化必须重新索权
    real = list(node.q())
    real[0] += 0.002
    node.send(real)
    s = wait_state(lambda s: not s["done"], 2.0, "D 重新索权")
    print(f"  D 发 +0.002 rad（> 死区）：{'✅ 重新索权（done=0）' if s else '❌ 没有索权'}")
    ok = ok and s is not None
    return 0 if ok else 1


def node_wait_controllers(node, names: list[str], timeout: float) -> bool:
    """在进程内调用 /controller_manager/list_controllers，直到 names 全部 active。

    为什么不用 `ros2 control list_controllers` 子进程：多实例 + 200 Hz 物理时，
    短命 CLI 节点经常收不到服务响应（日志 `failed to send response ... (timeout)`），
    于是工具会一直空等到超时，看起来像"控制器没起来"。
    """
    from controller_manager_msgs.srv import ListControllers

    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    if not client.wait_for_service(timeout_sec=10.0):
        return False
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if client.service_is_ready():
            future = client.call_async(ListControllers.Request())
            t_end = time.monotonic() + 3.0
            while not future.done() and time.monotonic() < t_end:
                rclpy.spin_once(node, timeout_sec=0.02)
            if future.done() and future.result() is not None:
                active = {c.name for c in future.result().controller if c.state == "active"}
                if all(n in active for n in names):
                    return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", default="b601_mit_arm",
                    help="要驱动的 MIT 控制器名（决定 __/joint_target 的话题前缀）")
    ap.add_argument("--controllers-file", default="",
                    help="控制器 yaml（默认用 b601_bringup 里的 MIT 那份）")
    ap.add_argument("--joints", default="joint1,joint2,joint3,joint4,joint5,joint6")
    ap.add_argument("--slew", type=float, default=0.5, help="目标斜坡速度（关节单位/s）")
    ap.add_argument("--hold", type=float, default=3.0, help="每个姿态到位后保持多久再采样")
    ap.add_argument("--settle-window", type=float, default=0.6, help="稳态采样窗口（s）")
    ap.add_argument("--tol", type=float, default=0.05, help="稳态误差判据（关节单位）")
    ap.add_argument("--max-vel", type=float, default=2.0, help="峰值角速度判据（关节单位/s）")
    ap.add_argument("--startup-timeout", type=float, default=40.0)
    ap.add_argument("--tag", default="run", help="CSV 文件名后缀")
    ap.add_argument("--csv", default="", help="CSV 输出路径（默认 .verify/sim_mit_ros_<tag>.csv）")
    ap.add_argument("--no-launch", action="store_true", help="外部已起 bringup 时用")
    ap.add_argument("--no-safety", action="store_true",
                    help="不启动安全节点。控制器回归**应该**用它：安全节点的速度/角度门限是为真机"
                         "设计的，仿真里开局若因重力前馈不准而下垂，会被它判急停，掩盖控制器的行为")
    ap.add_argument("--controllers", default="",
                    help="显式指定要 spawn 的控制器（空格分隔）；留空= broadcaster + --controller")
    ap.add_argument("--poses", default="",
                    help="自定义姿态序列：'a,b,c;d,e,f'（分号分隔多个姿态）。留空用默认手臂 3 姿态")
    ap.add_argument("--split", default="",
                    help="把手臂实例拆成多个（例：'5,1' = joint1-5 一个实例 + joint6 一个实例）。"
                         "验证「逐关节指定控制器」这个能力本身：两个实例必须能同时 active、"
                         "互不抢接口、各自跟踪自己那部分关节")
    ap.add_argument("--zero-gravity", action="store_true",
                    help="用零重力夹具跑（复制 MJCF 并把 gravity 设为 0）。"
                         "控制器实现回归用它：此时重力前馈必须同时关掉（--gravity-scale 0），"
                         "于是 τ = kp·err + kd·(−q̇) 是纯 PD，可与旧实现逐帧对比；"
                         "重力补偿准不准是另一条线（Phase 4），不要混在一起验")
    ap.add_argument("--mode", default="mit", choices=["mit", "pos_vel"],
                    help="mit=力控模式（话题发目标，本仓库自己算 τ）；"
                         "pos_vel=位置模式（JTC action 发轨迹，电机内部位置环）")
    ap.add_argument("--joint-friction", type=float, default=0.0,
                    help="给 MJCF 每个关节加库仑摩擦 frictionloss=N·m（真机有齿箱摩擦，"
                         "官方 MJCF 只有 viscous damping=0.02）——用来判断仿真失败是不是"
                         "『缺少静摩擦』造成的")
    ap.add_argument("--pure-pd", action="store_true",
                    help="纯 PD 配置（对标原 pos_vel 回路）：关重力前馈、关摩擦前馈、"
                         "并令 free_enabled=false —— 这样参考恒为上层目标、刚度恒为 kp_move，"
                         "不会进入『参考跟随实测』的等待模式（那个模式纯 PD 下等于自由下落）")
    ap.add_argument("--kp", default="", help="覆盖 kp_move，例 '60,60,60,20,10,8' 或单值")
    ap.add_argument("--kd", default="", help="覆盖 kd_move，例 '5.17,9.97,8.45,1.17,0.55,0.35' 或单值")
    ap.add_argument("--motor-impedance", action="store_true",
                    help="用电机侧阻抗（Phase 3，dm_command_mode:=mit_impedance）："
                         "位置环在驱动器里闭环，控制器只下发 pos/kp/kd/前馈")
    ap.add_argument("--temp-trip-test", action="store_true",
                    help="过温守护端到端：把失能线压到很低 + 加大热模型升温速率，"
                         "看「持续大扭矩 → 升温 → 过温失能（硬件 error）」是否真的发生")
    ap.add_argument("--temp-limit", type=float, default=27.0,
                    help="--temp-trip-test 用的失能线（℃）")
    ap.add_argument("--state-machine-test", action="store_true",
                    help="跑「指令层 + 整组 done + 变刚度」状态机专项测试（不看轨迹看时序）")
    ap.add_argument("--dump-interfaces", action="store_true",
                    help="启动后打印控制器状态与硬件接口归属（验证「每关节恰好一个属主」）")
    ap.add_argument("--hold-only", type=float, default=0.0,
                    help="不跑姿态，只在起始位形悬停 N 秒并报告漂移/峰值速度（用来量重力前馈是否撑得住）")
    ap.add_argument("--initial-poses", default=",".join(str(v) for v in SIM_INITIAL),
                    help="仿真起始位形（关节1..6，逗号分隔）。默认弯曲位形，避开竖直不稳定点")
    ap.add_argument("--gravity-scale", type=float, default=None,
                    help="启动后用 ros2 param set 设重力前馈系数（0 = 纯 PD，与真机 11-21 的测试口径一致）")
    args = ap.parse_args()

    joints = [j for j in args.joints.split(",") if j]
    project = Path(__file__).resolve().parent.parent
    if not args.controllers_file:
        # 默认用安装好的 MIT 控制器 yaml（不是位置模式那份！否则 controller_manager 里
        # 根本没有这个控制器的 type 声明，spawner 会报 "The 'type' param was not defined"）
        prefix = subprocess.run(
            ["ros2", "pkg", "prefix", "b601_bringup"],
            capture_output=True, text=True, check=False,
        ).stdout.strip()
        args.controllers_file = f"{prefix}/share/b601_bringup/config/b601_mit_controllers.yaml"
    csv_path = Path(args.csv) if args.csv else project / ".verify" / f"sim_mit_ros_{args.tag}.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    launch_log = project / ".verify" / f"sim_mit_ros_{args.tag}.launch.log"
    proc = None
    extra_controllers: list[str] = []
    if args.controllers:
        wanted = args.controllers.split()
        if wanted:
            args.controller = wanted[0]
            extra_controllers = wanted[1:]
    if not args.no_launch:
        cmds = [
            "ros2", "launch", "b601_bringup", "bringup.launch.py",
            "hardware:=mujoco", "use_rviz:=false",
            "dm_command_mode:=mit_torque",
            f"controllers:=joint_state_broadcaster {args.controller}",
        ]
        if args.controllers:
            cmds = [c for c in cmds if not c.startswith("controllers:=")]
            cmds.append(f"controllers:=joint_state_broadcaster {args.controllers}")

        if args.mode == "pos_vel":
            # 位置模式：controller_manager 用生成的位置模式 yaml，命令接口是 position
            cmds = [c for c in cmds if not c.startswith("controllers_file:=")
                    and not c.startswith("controllers:=")]
            cmds.append("dm_command_mode:=pos_vel")
            cmds.append("controllers:=joint_state_broadcaster b601_arm_controller b601_gripper_controller")
            args.controllers_file = ""
        if args.no_safety:
            cmds.append("use_safety:=false")
        if args.controllers_file:
            cmds.append(f"controllers_file:={args.controllers_file}")
        if args.split:
            import yaml as _yaml
            base_name = args.controller
            cfg = _yaml.safe_load(Path(args.controllers_file).read_text(encoding="utf-8")) or {}
            arm_params = (cfg.get(base_name) or {}).get("ros__parameters", {})
            all_joints = arm_params.get("joints", [])
            parts = [int(v) for v in args.split.split(",") if v]
            if sum(parts) != len(all_joints):
                print(f"❌ --split {args.split} 合计 {sum(parts)} ≠ joints {len(all_joints)}")
                return 1
            cm = cfg["controller_manager"]["ros__parameters"]
            cm.pop(base_name, None)
            cfg.pop(base_name, None)
            per_joint_keys = ("kp_hold", "kd_hold", "kp_move", "kd_move", "friction",
                              "torque_limit", "kp_free", "kd_free", "free_enabled",
                              "target_deadband", "settle_band_min", "settle_band_max")
            new_names = []
            offset = 0
            for idx, count in enumerate(parts):
                name = f"{base_name}_{idx + 1}"
                new_names.append(name)
                import copy as _copy
                params = {}
                for key, val in arm_params.items():
                    if key == "joints":
                        params[key] = all_joints[offset:offset + count]
                    elif key in per_joint_keys and isinstance(val, list) and len(val) == len(all_joints):
                        params[key] = val[offset:offset + count]
                    else:
                        params[key] = _copy.deepcopy(val)
                cm[name] = {"type": cm.get(base_name, {}).get("type", "b601_mit_controller/B601MitController")}
                if not cm[name]["type"]:
                    cm[name]["type"] = "b601_mit_controller/B601MitController"
                cfg[name] = {"ros__parameters": params}
                offset += count
            split_path = project / ".verify" / f"mit_split_{args.tag}.yaml"
            split_path.write_text(
                "# 回归工具生成：把手臂控制器拆成 %s\n" % args.split
                + _yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True), encoding="utf-8")
            cmds = [c for c in cmds if not c.startswith("controllers:=")]
            args.controllers_file = str(split_path)   # 后续 gravity 补丁要作用在这份上
            cmds.append(f"controllers:=joint_state_broadcaster {' '.join(new_names)}")
            args.controller = new_names[0]        # 主话题跟第一个实例走
            print(f"拆分实例：{new_names}（joints {all_joints}）")
            extra_controllers = new_names[1:]
        if args.pure_pd or args.kp or args.kd:
            import yaml as _yaml
            cfg = _yaml.safe_load(Path(args.controllers_file).read_text(encoding="utf-8")) or {}
            for key, val in (cfg or {}).items():
                if not (isinstance(val, dict) and "ros__parameters" in val):
                    continue
                rp = val["ros__parameters"]
                if "gravity_params" not in rp:
                    continue                      # 只改 MIT 实例
                if args.pure_pd:
                    rp["gravity_scale"] = 0.0
                    rp["use_gravity"] = False
                    rp["friction_scale"] = 0.0
                    n = len(rp.get("joints", []))
                    rp["free_enabled"] = [False] * n   # 关键：永不进入"参考跟随实测"
                def _vec(spec, old):
                    vals = [float(v) for v in spec.split(",") if v.strip()]
                    if len(vals) == 1:
                        return [vals[0]] * len(old)
                    return vals
                if args.kp:
                    rp["kp_move"] = _vec(args.kp, rp["kp_move"])
                    rp["kp_hold"] = list(rp["kp_move"])
                if args.kd:
                    rp["kd_move"] = _vec(args.kd, rp["kd_move"])
                    rp["kd_hold"] = list(rp["kd_move"])
            pd_path = project / ".verify" / f"mit_pd_{args.tag}.yaml"
            pd_path.write_text("# 回归工具生成：纯 PD（无重力补偿）\n"
                               + _yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True),
                               encoding="utf-8")
            args.controllers_file = str(pd_path)
        if args.motor_impedance:
            import yaml as _yaml
            cfg = _yaml.safe_load(Path(args.controllers_file).read_text(encoding="utf-8")) or {}
            for key, val in (cfg or {}).items():
                if isinstance(val, dict) and "ros__parameters" in val:
                    if "gravity_params" in val["ros__parameters"]:
                        val["ros__parameters"]["motor_impedance"] = True
            imp_path = project / ".verify" / f"mit_impedance_{args.tag}.yaml"
            imp_path.write_text(
                "# 回归工具生成：电机侧阻抗（motor_impedance=true）\n"
                + _yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True), encoding="utf-8")
            args.controllers_file = str(imp_path)
            cmds = [c for c in cmds if not c.startswith("dm_command_mode:=")]
            cmds.append("dm_command_mode:=mit_impedance")
        if args.temp_trip_test:
            import yaml as _yaml
            truth_path = subprocess.run(
                ["ros2", "pkg", "prefix", "b601_description"],
                capture_output=True, text=True, check=False,
            ).stdout.strip() + "/share/b601_description/config/b601_project.yaml"
            cfg = _yaml.safe_load(Path(truth_path).read_text(encoding="utf-8"))
            hw = cfg["hardware"]["dm"]
            hw["temperature"]["limit_c"] = args.temp_limit
            hw["temperature"]["warn_c"] = args.temp_limit - 1.0
            hw["thermal"]["ambient_c"] = args.temp_limit - 2.0
            hw["thermal"]["heat_c_per_s"] = 6.0    # 满载 6 ℃/s：几秒内就能到线
            patched = project / ".verify" / f"truth_temp_{args.tag}.yaml"
            patched.write_text(_yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True),
                               encoding="utf-8")
            cmds.append(f"project_config_file:={patched}")
        if args.joint_friction > 0.0:
            src = subprocess.run(
                ["ros2", "pkg", "prefix", "b601_mujoco"],
                capture_output=True, text=True, check=False,
            ).stdout.strip() + "/share/b601_mujoco/models/b601_arm.xml"
            text = Path(src).read_text(encoding="utf-8")
            patched, n_sub = re.subn(
                r'(<joint name="[^"]+" type="(?:hinge|slide)"[^/]*?)damping="[0-9.]+"',
                rf'\1frictionloss="{args.joint_friction}" damping="0.02"', text)
            if n_sub == 0:
                print("❌ 没能在 MJCF 里找到 joint 的 damping 属性，摩擦夹具未生效")
                return 1
            fr = project / ".verify" / f"b601_friction_{args.tag}.xml"
            fr.write_text(patched, encoding="utf-8")
            cmds.append(f"mujoco_model:={fr}")
            print(f"   已注入关节库仑摩擦 frictionloss={args.joint_friction} N·m（{n_sub} 个关节）")
        if args.zero_gravity:
            src = subprocess.run(
                ["ros2", "pkg", "prefix", "b601_mujoco"],
                capture_output=True, text=True, check=False,
            ).stdout.strip() + "/share/b601_mujoco/models/b601_arm.xml"
            text = Path(src).read_text(encoding="utf-8")
            patched = text.replace("<option ", '<option gravity="0 0 0" ', 1)
            if patched == text:
                print(f"❌ 没能在 {src} 里找到 <option> 标签，零重力夹具没生效")
                return 1
            zg = project / ".verify" / f"b601_zero_g_{args.tag}.xml"
            zg.write_text(patched, encoding="utf-8")
            cmds.append(f"mujoco_model:={zg}")
            if args.gravity_scale is None:
                args.gravity_scale = 0.0
        if args.initial_poses:
            import yaml as _yaml
            vals = [float(v) for v in args.initial_poses.split(",") if v]
            names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
                     "finger_left", "finger_right"]
            ip = {n: (vals[i] if i < len(vals) else 0.0) for i, n in enumerate(names)}
            ip_path = project / ".verify" / f"sim_initial_{args.tag}.yaml"
            ip_path.write_text(
                "# 仿真专用起始位形（回归工具生成，不改真值）\n"
                + _yaml.safe_dump({"initial_positions": ip}, sort_keys=False),
                encoding="utf-8")
            cmds.append(f"initial_positions_file:={ip_path}")
        if args.gravity_scale is not None:
            import yaml as _yaml
            cfg = _yaml.safe_load(Path(args.controllers_file).read_text(encoding="utf-8")) or {}
            for key, val in (cfg or {}).items():
                if isinstance(val, dict) and "ros__parameters" in val:
                    if "gravity_params" in val["ros__parameters"]:
                        val["ros__parameters"]["gravity_scale"] = float(args.gravity_scale)
            patched_yaml = project / ".verify" / f"mit_ctrl_{args.tag}.yaml"
            patched_yaml.write_text(
                "# 回归工具生成：把 gravity_scale 固定为 %s\n" % args.gravity_scale
                + _yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True),
                encoding="utf-8")
            args.controllers_file = str(patched_yaml)
        # controllers_file 只追加一次（拆分/gravity 补丁都可能改过它）
        cmds = [c for c in cmds if not c.startswith("controllers_file:=")]
        if args.controllers_file:
            cmds.append(f"controllers_file:={args.controllers_file}")
        env = dict(os.environ)
        env.setdefault("ROS_HOME", str(project / ".verify" / "roshome"))
        env.setdefault("ROS_LOG_DIR", str(project / ".verify" / "roslog"))
        def _spawn() -> subprocess.Popen:
            return subprocess.Popen(
                cmds, stdout=launch_log.open("w"), stderr=subprocess.STDOUT,
                env=env, start_new_session=True,
            )

        proc = _spawn()

    def cleanup() -> None:
        if proc is None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            return
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and proc.poll() is None:
            time.sleep(0.2)
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

    rclpy.init()
    node = Harness(args.controller, joints)
    arm_client = node.arm_action_client("b601_arm_controller") if args.mode == "pos_vel" else None
    for name in extra_controllers:
        node.extra_pubs.append(
            node.create_publisher(JointState, f"/{name}/joint_target", 10))
    rc = 0
    try:
        if proc is not None:
            if args.mode == "pos_vel":
                args.controller = "b601_arm_controller"
                extra_controllers = ["b601_gripper_controller"]
            ctrl_names = [args.controller] + [n for n in extra_controllers]
            ok_up = node_wait_controllers(
                node, ["joint_state_broadcaster"] + ctrl_names, args.startup_timeout)
            if not ok_up:
                # spawner 会偶发拿不到 /controller_manager/list_controllers 服务（DDS 抖动，
                # 连续多次 launch 时更容易出现）→ 清场后整条重试一次
                print("   ⚠ 控制器没起来，清理后重试一次（spawner 偶发 DDS 抖动）")
                cleanup()
                time.sleep(3.0)
                proc = _spawn()
                ok_up = node_wait_controllers(
                    node, ["joint_state_broadcaster"] + ctrl_names, args.startup_timeout)
            if not ok_up:
                print(f"❌ 控制器没能 active（{args.startup_timeout:.0f}s 超时）；日志：{launch_log}")
                tail = launch_log.read_text(encoding="utf-8", errors="replace").splitlines()[-25:]
                for line in tail:
                    print("   " + line)
                return 1
        if not node.wait_ready(20.0):
            print("❌ 收不到 /joint_states")
            return 1
        if args.gravity_scale is not None:
            out = subprocess.run(
                ["ros2", "param", "set", f"/{args.controller}", "gravity_scale",
                 f"{args.gravity_scale}"],
                capture_output=True, text=True, timeout=15, check=False,
            )
            print(f"gravity_scale := {args.gravity_scale}（启动前已写入控制器 yaml；"
                  f"此处再 set 一次做确认：{out.stdout.strip() or out.stderr.strip()}）")
            node.spin_for(0.5)

        if args.temp_trip_test:
            print(f"过温测试：失能线 {args.temp_limit:.0f} ℃，热模型满载 6 ℃/s（需真实重力负载）")
            node.spin_for(1.0)
            hold = node.q()
            end = time.monotonic() + 20.0
            tripped = False
            while time.monotonic() < end:
                node.send(hold)          # 持续要"保持"，控制器会用 τ 顶重力 → 发热
                node.spin_for(0.2)
                text = launch_log.read_text(encoding="utf-8", errors="replace") \
                    if launch_log.exists() else ""
                if "过温保护" in text:
                    tripped = True
                    break
            print(f"  {'✅ 过温保护已触发（硬件置 error、总线失能）' if tripped else '❌ 20 s 内没触发'}")
            if tripped:
                for line in launch_log.read_text(encoding="utf-8", errors="replace").splitlines():
                    if "过温保护" in line or "温度" in line:
                        print("   " + line.strip()[-160:])
            print("  RESULT " + json.dumps({"mode": "temp_trip", "tripped": tripped,
                                            "pass": tripped}))
            return 0 if tripped else 1
        if args.state_machine_test:
            rc = state_machine_test(node, joints, args)
            print(f"  RESULT " + json.dumps({"mode": "state_machine", "pass": rc == 0}))
            return rc
        if args.dump_interfaces:
            for cmd in (["ros2", "control", "list_controllers"],
                        ["ros2", "control", "list_hardware_interfaces"]):
                out = subprocess.run(cmd, capture_output=True, text=True, timeout=20, check=False).stdout
                print(f"--- {' '.join(cmd[2:])} ---")
                for line in out.splitlines():
                    if "effort" in line or "controller" in line.lower() or "active" in line:
                        print("   " + line.strip())
        if args.hold_only > 0.0:
            q_start = node.q()
            node.samples.clear()
            node.spin_for(args.hold_only)
            q_end = node.q()
            drift = [q_end[i] - q_start[i] for i in range(len(joints))]
            peaks_h = [0.0] * len(joints)
            for s in node.samples:
                for i in range(len(joints)):
                    peaks_h[i] = max(peaks_h[i], abs(s[2][i]))
            max_drift = max(abs(d) for d in drift)
            max_vel = max(peaks_h)
            print(f"  悬停 {args.hold_only:.1f}s 漂移 = {json.dumps([round(d, 4) for d in drift])} "
                  f"→ 最大 {max_drift:.4f}")
            print(f"  峰值 |q̇| = {json.dumps([round(v, 3) for v in peaks_h])} → 最大 {max_vel:.3f}")
            if node.first_debug:
                n = len(joints)
                print(f"  首帧 τ_g = {json.dumps([round(v, 6) for v in node.first_debug[n:2*n]])}  "
                      f"（q={json.dumps([round(v, 6) for v in node.first_debug_q])}）")
            if self_last := node.last_debug:
                n = len(joints)
                print(f"  控制器 debug: τ={json.dumps([round(v,3) for v in self_last[0:n]])}")
                print(f"                τ_g={json.dumps([round(v,3) for v in self_last[n:2*n]])}")
                print(f"                kp={json.dumps([round(v,2) for v in self_last[2*n:3*n]])}")
                print(f"                err={json.dumps([round(v,4) for v in self_last[3*n:4*n]])}")
                print(f"  q_end={json.dumps([round(v,4) for v in q_end])}")
            print("  RESULT " + json.dumps({
                "mode": "hold_only", "controller": args.controller,
                "drift": [round(d, 4) for d in drift], "max_drift": round(max_drift, 4),
                "peak_vel_max": round(max_vel, 4),
                "pass": max_drift <= args.tol and max_vel <= args.max_vel,
            }, ensure_ascii=False))
            return 0 if (max_drift <= args.tol and max_vel <= args.max_vel) else 1

        peaks = [0.0] * len(joints)
        results = []
        node.spin_for(1.0)      # 让 /joint_states 先稳一下，记录起始位形
        q0 = node.q()
        print(f"起始位形：{json.dumps([round(v, 4) for v in q0])}")

        poses = DEFAULT_POSES
        if args.poses:
            poses = [[float(v) for v in ps.split(",")] for ps in args.poses.split(";") if ps.strip()]
        for idx, target in enumerate(poses):
            print(f"\n== 姿态 {idx + 1}/{len(poses)}: {json.dumps([round(v, 3) for v in target])}")
            node.samples.clear()
            if args.mode == "pos_vel":
                node.send_ramp_trajectory(target, args.slew, arm_client)
            else:
                node.ramp_to(target, args.slew, tag=f"pose{idx + 1}")
            node.spin_for(args.hold)
            # 稳态：取最后 settle_window 的平均
            now = time.monotonic()
            window = [s for s in node.samples if s[0] >= now - args.settle_window]
            if not window:
                print("   ❌ 没有采样")
                rc = 1
                continue
            q_mean = [sum(s[1][i] for s in window) / len(window) for i in range(len(joints))]
            err = [target[i] - q_mean[i] for i in range(len(joints))]
            for s in node.samples:
                for i in range(len(joints)):
                    peaks[i] = max(peaks[i], abs(s[2][i]))
            max_err = max(abs(e) for e in err)
            ok = max_err <= args.tol
            if not ok:
                rc = 1
            results.append({"pose": idx + 1, "max_err": max_err, "ok": ok})
            print(f"   稳态 q = {json.dumps([round(v, 4) for v in q_mean])}")
            print(f"   误差   = {json.dumps([round(e, 4) for e in err])} → 最大 {max_err:.4f} "
                  f"（判据 {args.tol}）{'✅' if ok else '❌'}")

        peak_max = max(peaks) if peaks else 0.0
        if peak_max > args.max_vel:
            rc = 1
        level_name = {0: "NORMAL", 1: "WARNING", 2: "EMERGENCY_STOP"}.get(node.status_level, str(node.status_level))
        print("\n== 汇总")
        print(f"  峰值 |q̇| = {json.dumps([round(v, 3) for v in peaks])} → 最大 {peak_max:.3f} "
              f"（判据 {args.max_vel}）{'✅' if peak_max <= args.max_vel else '❌'}")
        print(f"  安全状态 = {level_name} {node.status_text}")
        estop = node.status_level >= 2
        if estop:
            rc = 1
        print("  RESULT " + json.dumps({
            "mode": args.mode,
            "controller": args.controller,
            "poses": results,
            "peak_vel": [round(v, 4) for v in peaks],
            "peak_vel_max": round(peak_max, 4),
            "estop": estop,
            "status": level_name,
            "pass": rc == 0,
        }, ensure_ascii=False))

        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["t_rel", *[f"{j}_pos" for j in joints], *[f"{j}_vel" for j in joints]])
            if node.samples:
                t0 = node.samples[0][0]
                for t, pos, vel in node.samples:
                    writer.writerow([round(t - t0, 4), *[round(v, 6) for v in pos],
                                     *[round(v, 6) for v in vel]])
        print(f"  CSV: {csv_path}")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        cleanup()
    return rc


if __name__ == "__main__":
    sys.exit(main())
