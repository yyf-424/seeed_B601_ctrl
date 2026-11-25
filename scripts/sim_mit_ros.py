#!/usr/bin/env python3
"""sim_mit_ros.py —— 在 MuJoCo 后端上端到端验证 MIT 力控控制器（ROS 全链路）

为什么要有它：
    `scripts/sim_mit_check.py` / `sim_mit_poses.py` 都是**纯 MuJoCo + Python 重写控制律**，
    验证的是"控制律的数学"；本脚本走真正的 ROS 链路（controller_manager → 控制器 →
    MuJoCo 硬件插件 → 安全节点），验证的是"控制器实现"。改控制器代码时必须用它做回归。

它做什么：
    1) 起 `bringup.launch.py hardware:=mujoco dm_command_mode:=mit_impedance`（可指定控制器 yaml）；
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
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from b601_demos.target_client import TargetClient
from builtin_interfaces.msg import Duration as MsgDuration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from project_config import load_project_config
from sync_config import MIT_ARRAY_KEYS

PROJECT = load_project_config()
SETTINGS = PROJECT["tools"]["sim_mit_ros"]

# 仿真专用起始位形：真值的 startup 是"竖直位"（真机上电姿态），那是**不稳定平衡点** ——
# 仿真里重力前馈稍有偏差就会倾倒，一开局就把手臂甩到 6 rad/s。所以回归用弯曲位形。
SIM_INITIAL = SETTINGS["sim_initial"]

DEFAULT_POSES = SETTINGS["default_poses"]


class Harness(Node):
    def __init__(self, controller: str, arm_joints: list[str], source="sim_regression"):
        super().__init__("sim_mit_ros_harness")
        self.arm_joints = arm_joints
        self.latest: dict[str, float] = {}
        self.latest_vel: dict[str, float] = {}
        self.stamp = 0.0
        self.samples: list[tuple[float, list[float], list[float]]] = []
        self.status_level = -1
        self.status_text = ""
        self.limit_profile = None
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
        self.targets = TargetClient(self, source)
        self.task_pub = self.create_publisher(Float64MultiArray, f"/{controller}/task_torque", 10)
        self.ff: list[float] = []
        self.create_subscription(Float64MultiArray, f"/{controller}/ff", self._on_ff, 10)
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

    def _on_ff(self, msg) -> None:
        self.ff = list(msg.data)

    def _on_status(self, msg) -> None:
        level = msg.level
        if isinstance(level, (bytes, bytearray)):
            level = level[0] if level else 0
        self.status_level = int(level)
        self.status_text = msg.message
        self.limit_profile = next((v.value for v in msg.values if v.key == "limit_profile"), None)

    # ------------------------------------------------------------------ 工具
    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=SETTINGS["executor_poll_sec"])

    def wait_ready(self, timeout: float) -> bool:
        """等到 /joint_states 有本实例全部关节。"""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=SETTINGS["ready_poll_sec"])
            if self.arm_joints and all(j in self.latest for j in self.arm_joints):
                return True
        return False

    def q(self) -> list[float]:
        return [self.latest[j] for j in self.arm_joints]

    def send(self, target: list[float], velocity: list[float] | None = None) -> None:
        msg = JointState()
        msg.name = list(self.arm_joints)
        msg.position = list(target)
        if velocity is not None:
            msg.velocity = velocity
        self.targets.publish(msg)

    def send_ramp_trajectory(self, target: list[float], slew: float, action_client,
                             result_timeout: float = SETTINGS["action_result_timeout_sec"]) -> bool:
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
        if not action_client.wait_for_server(timeout_sec=SETTINGS["action_request_timeout_sec"]):
            print("   ❌ 找不到 JTC action server（位置模式控制器在跑吗）")
            return False
        send_future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=SETTINGS["action_request_timeout_sec"])
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

    def ramp_to(self, target: list[float], slew: float, rate_hz: float = SETTINGS["publish_rate_hz"], tag: str = "") -> None:
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

    检查高低刚度切换、整组到位和相同目标的重发。
    """
    fixture = SETTINGS["state_machine"]
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters

    client = node.create_client(GetParameters, f"/{args.controller}/get_parameters")
    if not client.wait_for_service(timeout_sec=fixture["parameter_timeout_sec"]):
        print("❌ 读不到控制器的 settle_band_min 参数服务")
        return 1
    future = client.call_async(GetParameters.Request(
        names=["kp_move", "kp_free", "settle_band_max"]))
    rclpy.spin_until_future_complete(node, future, timeout_sec=fixture["parameter_timeout_sec"])
    response = future.result() if future.done() else None
    if response is None or not response.values:
        print("❌ 读不到控制器的 settle_band_min")
        return 1
    def first_joint(value):
        if value.type == ParameterType.PARAMETER_DOUBLE:
            return value.double_value
        if value.type == ParameterType.PARAMETER_DOUBLE_ARRAY and value.double_array_value:
            return value.double_array_value[0]
        raise ValueError("预期逐关节浮点参数")

    try:
        kp_move, kp_free, max_band = map(first_joint, response.values)
    except ValueError as error:
        print(f"❌ {error}")
        return 1
    kp_threshold = kp_move * fixture["kp_fraction"]

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
            node.spin_for(SETTINGS["executor_poll_sec"])
            s = snapshot()
            if s is not None and pred(s):
                return s
        print(f"   ❌ {label} 超时（{timeout:.1f}s）")
        return None

    ok = True
    node.spin_for(fixture["warmup_sec"])
    s = snapshot()
    if s is None or s["done"] or abs(s["kp"][0] - kp_move) > fixture["kp_free_tolerance"]:
        print("❌ 未收到首个目标时应继续使用高刚度")
        return 1
    print(f"  A 首个目标前保持高刚度，joint1 kp={s['kp'][0]:g}")

    base = node.q()
    target = list(base)
    target[0] += fixture["step_rad"]
    print(f"  B 发目标 joint1+{fixture['step_rad']:g} rad …")
    t0 = time.monotonic()
    seq = s["seq"]
    node.send(target)
    s = wait_state(lambda s: s["seq"] > seq and not s["done"], fixture["claim_timeout_sec"], "进入跟踪")
    if s is None:
        ok = False
    else:
        print(f"     done=0 用时 {time.monotonic() - t0:.3f}s；band={s['band'][0]:.4f}")
    s = wait_state(lambda s: s["kp"][0] > kp_threshold, fixture["kp_rise_timeout_sec"], "kp 升到 80%")
    if s is None:
        ok = False
    else:
        print(f"     kp 升到 {kp_threshold:g} 用时 {time.monotonic() - t0:.3f}s")
    s = wait_state(lambda s: s["done"], fixture["settle_timeout_sec"], "done→1（到位锁存）")
    if s is None:
        ok = False
    else:
        print(f"     整组 done 用时 {time.monotonic() - t0:.2f}s；此时 err={json.dumps([round(e, 4) for e in s['err_cmd']])}")
    s = wait_state(lambda s: abs(s["kp"][0] - kp_free) <= fixture["kp_free_tolerance"], fixture["kp_free_timeout_sec"], "kp 落回 kp_free")
    if s is None:
        ok = False
    else:
        print(f"     kp 回落到 {s['kp'][0]:.2f} 用时 {time.monotonic() - t0:.2f}s（含 3 s 到位判定）")

    # B2：误差带上限（大位移时 5% 会被 cap 到 settle_band_max=0.05）
    big = target.copy()
    big[0] += fixture["large_step_rad"]                       # joint1 走 2 rad（限位 ±2.6）：5% = 0.1 > cap 0.05
    node.send(big)
    s = wait_state(lambda s: (not s["done"]), fixture["claim_timeout_sec"], "B2 索权")
    if s is not None:
        cap_ok = abs(s["band"][0] - max_band) < 1e-6
        print(f"  B2 大位移 {fixture['large_step_rad']:g} rad → band={s['band'][0]:.4f}（cap {max_band:g}）"
              f"{'✅' if cap_ok else '❌'}")
        ok = ok and cap_ok
    s = wait_state(lambda s: s["done"], fixture["settle_timeout_sec"] + fixture["large_step_rad"] / PROJECT["target_slew"]["revolute"], "B2 到位")
    if s is None:
        ok = False

    # B3：整组 done 由**最慢**关节决定（joint1 只动 0.05、joint3 动 0.6）
    mix = big.copy()
    for joint, step in fixture["mixed_steps_rad"].items():
        mix[joints.index(joint)] += step
    t3 = time.monotonic()
    node.send(mix)
    if wait_state(lambda s: (not s["done"]), fixture["claim_timeout_sec"], "B3 索权") is None:
        ok = False
    s = wait_state(lambda s: s["done"], fixture["mixed_settle_timeout_sec"], "B3 整组 done")
    if s is None:
        ok = False
    else:
        el = time.monotonic() - t3
        # joint3 的目标位移更大，整组到位必须等这个关节跟上。
        slow_ok = el > fixture["mixed_min_sec"]
        print(f"  B3 混合目标→ 整组 done 用时 {el:.2f}s "
              f"{'✅ 由慢关节决定' if slow_ok else '❌ 偏快'}")
        ok = ok and slow_ok

    # 重发 B3 的原目标，等控制器处理这一帧，再检查是否重新跟踪。
    if s is None:
        return 1
    seq = s["seq"]
    node.send(mix)
    repeated = wait_state(lambda state: state["seq"] > seq, fixture["claim_timeout_sec"], "接收重发目标")
    unchanged = repeated is not None and repeated["done"]
    print(f"  相同目标重发，{'保持到位' if unchanged else '重新跟踪或超时'}")
    ok = ok and unchanged
    tiny = mix.copy()
    tiny[0] += fixture["small_step_rad"]
    node.send(tiny)
    changed = wait_state(lambda state: not state["done"], fixture["claim_timeout_sec"], "微小目标进入跟踪")
    print(f"  微小目标 {fixture['small_step_rad']:g} rad，{'已接受' if changed else '未接受'}")
    ok = ok and changed is not None
    return 0 if ok else 1


def node_wait_controllers(node, names: list[str], timeout: float) -> bool:
    """在进程内调用 /controller_manager/list_controllers，直到 names 全部 active。

    为什么不用 `ros2 control list_controllers` 子进程：多实例 + 200 Hz 物理时，
    短命 CLI 节点经常收不到服务响应（日志 `failed to send response ... (timeout)`），
    于是工具会一直空等到超时，看起来像"控制器没起来"。
    """
    from controller_manager_msgs.srv import ListControllers

    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    if not client.wait_for_service(timeout_sec=SETTINGS["controller_service_timeout_sec"]):
        return False
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if client.service_is_ready():
            future = client.call_async(ListControllers.Request())
            t_end = time.monotonic() + SETTINGS["controller_response_timeout_sec"]
            while not future.done() and time.monotonic() < t_end:
                rclpy.spin_once(node, timeout_sec=SETTINGS["executor_poll_sec"])
            if future.done() and future.result() is not None:
                active = {c.name for c in future.result().controller if c.state == "active"}
                if all(n in active for n in names):
                    return True
        rclpy.spin_once(node, timeout_sec=SETTINGS["ready_poll_sec"])
    return False


def mit_frame_test(node: Harness) -> int:
    """在零重力夹具上，用反馈验证速度目标，并观测三段力矩的合成与超时。"""
    from std_msgs.msg import Float64MultiArray

    fixture = SETTINGS["mit_frame"]
    node.spin_for(fixture["warmup_sec"])
    position = node.q()
    n = len(position)
    node.send(position)
    node.spin_for(fixture["hold_sec"])
    for _ in range(fixture["velocity_samples"]):
        node.send(position, [fixture["velocity_rad_s"]] + [0.0] * (n - 1))
        node.spin_for(fixture["velocity_cadence_sec"])
    velocity_displacement = node.q()[0] - position[0]
    node.spin_for(fixture["velocity_timeout_wait_sec"])
    after_timeout = abs(node.q()[0] - position[0])

    # 任务力矩在发送窗口内有效，过期后保留位置目标和其他前馈。
    node.task_pub.publish(Float64MultiArray(data=[fixture["task_torque_nm"]] + [0.0] * (n - 1)))
    node.spin_for(fixture["task_observation_sec"])
    task_applied = node.ff[2 * n] if len(node.ff) == 3 * n else None
    combined = node.state[0] if len(node.state) == 8 * n + 4 else None
    node.spin_for(fixture["task_timeout_wait_sec"])
    task_expired = len(node.ff) == 3 * n and abs(node.ff[2 * n]) < 1e-8
    gravity = node.ff[n:2 * n]
    gravity_nonzero = any(abs(x) > fixture["minimum_gravity_nm"] for x in gravity)

    # joint2 的测试用摩擦系数在实际运动时产生前馈。
    position[1] -= fixture["friction_motion_rad"]
    node.send(position)
    friction_peak = 0.0
    for _ in range(10):
        node.spin_for(fixture["friction_sample_sec"])
        if len(node.ff) == 3 * n:
            friction_peak = max(friction_peak, abs(node.ff[1]))

    passed = (
        velocity_displacement > fixture["displacement_threshold_rad"]
        and after_timeout < fixture["displacement_threshold_rad"]
        and task_applied is not None and abs(task_applied - fixture["task_torque_nm"]) < 1e-6
        and combined is not None and abs(combined - fixture["task_torque_nm"]) < 1e-6
        and task_expired and gravity_nonzero and friction_peak > fixture["minimum_friction_nm"]
    )
    print("  RESULT " + json.dumps({
        "mode": "mit_frame", "velocity_displacement_rad": velocity_displacement,
        "after_velocity_timeout_rad": after_timeout, "task_torque_nm": task_applied,
        "combined_joint1_nm": combined, "task_expired": task_expired,
        "gravity_nm": gravity, "friction_peak_nm": friction_peak, "pass": passed,
    }))
    return 0 if passed else 1


def fixture_gravity_params(model_path: Path) -> list[float]:
    """用测试 MJCF 的质量和质心生成夹具前馈，真机辨识参数仍来自统一配置。"""
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(model_path))
    links = [name for name in PROJECT["model"]["links"] if name not in ("base_link", "link1")]
    params = []
    for name in links:
        body = model.body(name)
        mass = float(body.mass[0])
        params.extend([mass, *(float(mass * component) for component in body.ipos)])
    return params


def gravity_varstiff_switch_test(node: Harness, csv_path: Path) -> int:
    """用真实控制器验证升档、到位、降档以及逐周期实测保持。"""
    fixture = SETTINGS["gravity_varstiff_switch"]
    mit = PROJECT["controllers"]["mit"]
    arm = mit["instances"]["b601_mit_arm"]
    profile = mit["profiles"][arm["gravity_profile"]]
    pure_pd = mit["profiles"][arm["profile"]]["group"]
    count = len(arm["joints"])
    gains = [profile["per_joint"][joint] for joint in arm["joints"]]
    move_gains = {kind: [gain[f"{kind}_move"] for gain in gains] for kind in ("kp", "kd")}
    hold_gains = {
        kind: [gain * profile["hold_gain_ratios"][kind] for gain in values]
        for kind, values in move_gains.items()
    }
    stages = []

    def ready():
        return node.status_level in (0, 1) and len(node.state) == 8 * count + 4

    def wait_for(condition):
        end = time.monotonic() + fixture["motion_timeout_sec"]
        while time.monotonic() < end and node.status_level < 2:
            node.spin_for(SETTINGS["executor_poll_sec"])
            if ready() and condition(node.state):
                return True
        return False

    def at_gains(state, stage):
        expected = hold_gains if stage == "free" else move_gains
        return all(
            abs(actual - gain) <= fixture["gain_tolerance"]
            for kind, offset in (("kp", 2), ("kd", 3))
            for actual, gain in zip(state[offset * count:(offset + 1) * count], expected[kind])
        )

    def gravity_present(frames):
        # 此档摩擦和任务项为零，因此 effort 目标就是实际重力前馈。
        for _, state in frames:
            scales = ([profile["group"]["gravity_joint_scale_free"]] * count
                      if state[8 * count] else [gain["gravity_joint_scale"] for gain in gains])
            if max(abs(value) for value in state[:count]) < fixture["minimum_signal_nm"]:
                return False
            if any(abs(torque + scale * raw) > fixture["masked_torque_tolerance_nm"]
                   for torque, raw, scale in zip(state[:count], state[count:2 * count], scales)):
                return False
        return bool(frames)

    def angular_travel(samples):
        return [max(values) - min(values) for values in
                zip(*(position for _, position, _ in samples))]

    def check_hold(label):
        handoff_sample_index = len(node.samples)
        handoff_state_index = len(node.state_log) - 1
        if not wait_for(lambda state: bool(state[8 * count]) and at_gains(state, "free")
                        and node.limit_profile == "low_stiffness"):
            stages.append({"stage": label, "reason": "低刚度未就绪", "pass": False})
            return False
        sample_index = len(node.samples)
        state_index = len(node.state_log) - 1
        deadline = time.monotonic() + fixture["motion_timeout_sec"]
        duration = 0.0
        while duration < fixture["hold_sec"] and time.monotonic() < deadline and ready():
            node.spin_for(SETTINGS["executor_poll_sec"])
            if len(node.samples) > sample_index:
                duration = node.samples[-1][0] - node.samples[sample_index][0]
        samples = node.samples[sample_index:]
        states = node.state_log[state_index:]
        travel = angular_travel(samples)
        handoff_travel = []
        if samples:
            start = node.samples[handoff_sample_index][0]
            handoff_samples = [sample for sample in node.samples[handoff_sample_index:]
                               if sample[0] - start <= fixture["hold_sec"]]
            handoff_travel = angular_travel(handoff_samples)
        reference_states = node.state_log[handoff_state_index:]
        reference_error = max((abs(value) for _, state in reference_states
                               for value in state[5 * count:6 * count]), default=None)
        passed = (ready() and node.limit_profile == "low_stiffness"
                  and duration >= fixture["hold_sec"] and len(travel) == count
                  and max(travel + handoff_travel) <= fixture["hold_max_travel_rad"]
                  and reference_error == 0.0
                  and all(bool(state[8 * count]) and at_gains(state, "free") for _, state in states)
                  and gravity_present(reference_states))
        stages.append({"stage": label, "sample_span_sec": duration, "samples": len(samples),
                       "travel_rad": travel, "handoff_travel_rad": handoff_travel,
                       "reference_error_rad": reference_error,
                       "gravity_present": gravity_present(reference_states),
                       "limit_profile": node.limit_profile, "pass": passed})
        return passed

    start = wait_for(lambda state: not state[8 * count] and at_gains(state, "move")
                     and node.limit_profile == "normal")
    before_start = len(node.state_log) - 1
    if start:
        node.spin_for(profile["group"]["settle_sec"] + profile["group"]["kp_down_sec"])
    startup = node.state_log[max(0, before_start):]
    startup_pass = (start and ready() and gravity_present(startup) and
                    all(not state[8 * count] and at_gains(state, "move") for _, state in startup))
    stages.append({"stage": "startup_high", "pass": startup_pass})
    for index, delta in enumerate(fixture["moves_rad"], start=1):
        if not startup_pass or not ready():
            break
        target = [q + step for q, step in zip(node.q(), delta)]
        sequence = node.state[8 * count + 1]
        state_index = len(node.state_log)
        node.send(target)
        high = wait_for(lambda state: state[8 * count + 1] > sequence
                        and not state[8 * count] and at_gains(state, "move")
                        and node.limit_profile == "normal")
        done = high and wait_for(lambda state: bool(state[8 * count]))
        high_frames = [(stamp, state) for stamp, state in node.state_log[state_index:]
                       if state[8 * count + 1] > sequence and not state[8 * count]
                       and at_gains(state, "move")]
        settled = [(stamp, state) for stamp, state in high_frames
                   if 0.0 < state[8 * count + 2]]
        if settled:
            end = settled[-1][0]
            window = fixture["settle_window_sec"]
            settled = [(stamp, state) for stamp, state in settled if end - stamp <= window]
        errors = [max(abs(state[4 * count + i]) for _, state in settled)
                  for i in range(count)] if settled else []
        band_min = pure_pd["settle_band_min"]
        band_max = pure_pd["settle_band_max"]
        band = [min(band_max, max(band_min, abs(step) * pure_pd["settle_band_pct"]))
                for step in delta]
        controller_band = high_frames[-1][1][6 * count:7 * count] if high_frames else []
        passed = (done and len(errors) == count and len(controller_band) == count
                  and gravity_present(high_frames)
                  and all(band_min <= limit <= band_max for limit in controller_band)
                  and all(error <= limit for error, limit in zip(errors, band)))
        stages.append({"stage": f"move_{index}", "target_rad": target, "band_rad": band,
                       "controller_band_rad": controller_band,
                       "error_rad": errors, "gravity_present": gravity_present(high_frames),
                       "high_gains_reached": high, "done": done, "pass": passed})
        if not done:
            break
        check_hold(f"hold_{index}")

    expected = 1 + 2 * len(fixture["moves_rad"])
    passed = len(stages) == expected and all(stage["pass"] for stage in stages)
    result = {"mode": "gravity_compensation", "fixture_gravity": "mujoco_model_inertias",
              "joints": arm["joints"], "stages": stages,
              "safety_level": node.status_level, "safety_status": node.status_text,
              "pass": passed}
    csv_path.with_suffix(".json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n",
                                            encoding="utf-8")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["t_rel", *[f"{j}_pos" for j in arm["joints"]],
                         *[f"{j}_vel" for j in arm["joints"]]])
        if node.samples:
            start = node.samples[0][0]
            writer.writerows((stamp - start, *q, *vel) for stamp, q, vel in node.samples)
    for stage in stages:
        print("  RESULT " + json.dumps(stage, ensure_ascii=False))
    print(f"  安全状态 {node.status_level} {node.status_text}，记录 {csv_path.with_suffix('.json')}")
    return 0 if passed else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", default="b601_mit_arm",
                    help="要驱动的 MIT 控制器名（决定仲裁控制器组和诊断话题前缀）")
    ap.add_argument("--controllers-file", default="",
                    help="控制器 yaml（默认用 b601_bringup 里的 MIT 那份）")
    ap.add_argument("--gravity-varstiff-switch-test", action="store_true",
                    help="使用正式高低增益与 MuJoCo 标称重力验证自动切换")
    ap.add_argument("--joints", default=",".join(PROJECT["robot"]["arm_joints"]))
    ap.add_argument("--slew", type=float, default=SETTINGS["slew"], help="目标斜坡速度（关节单位/s）")
    ap.add_argument("--hold", type=float, default=SETTINGS["hold"], help="每个姿态到位后保持多久再采样")
    ap.add_argument("--settle-window", type=float, default=SETTINGS["settle_window"], help="稳态采样窗口（s）")
    ap.add_argument("--tol", type=float, default=SETTINGS["tol"], help="稳态误差判据（关节单位）")
    ap.add_argument("--max-vel", type=float, default=SETTINGS["max_vel"], help="峰值角速度判据（关节单位/s）")
    ap.add_argument("--startup-timeout", type=float, default=SETTINGS["startup_timeout"])
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
                    help="mit=电机侧 MIT（话题发位置和速度目标）；"
                         "pos_vel=位置模式（JTC action 发轨迹，电机内部位置环）")
    ap.add_argument("--joint-friction", type=float, default=SETTINGS["joint_friction"],
                    help="给 MJCF 每个关节加库仑摩擦 frictionloss=N·m（真机有齿箱摩擦，"
                         "官方 MJCF 只有 viscous damping=0.02）——用来判断仿真失败是不是"
                         "『缺少静摩擦』造成的")
    ap.add_argument("--pure-pd", action="store_true",
                    help="纯 PD 配置（对标原 pos_vel 回路）：关重力前馈、关摩擦前馈、"
                         "并令 free_enabled=false —— 这样参考恒为上层目标、刚度恒为 kp_move，"
                         "不会进入『参考跟随实测』的等待模式（那个模式纯 PD 下等于自由下落）")
    ap.add_argument("--kp", default="", help="覆盖仿真跟踪 Kp，使用逗号分隔的逐轴值或单值")
    ap.add_argument("--kd", default="", help="覆盖仿真跟踪 Kd，使用逗号分隔的逐轴值或单值")
    ap.add_argument("--motor-impedance", action="store_true",
                    help="兼容参数，MIT 现已默认运行电机侧阻抗")
    ap.add_argument("--temp-trip-test", action="store_true",
                    help="过温守护端到端：把失能线压到很低 + 加大热模型升温速率，"
                         "看「持续大扭矩 → 升温 → 过温失能（硬件 error）」是否真的发生")
    ap.add_argument("--temp-limit", type=float, default=SETTINGS["temp_limit"],
                    help="--temp-trip-test 用的失能线（℃）")
    ap.add_argument("--mit-frame-test", action="store_true",
                    help="验证 MIT 速度目标和三段前馈（使用零重力夹具）")
    ap.add_argument("--state-machine-test", action="store_true",
                    help="跑「指令层 + 整组 done + 变刚度」状态机专项测试（不看轨迹看时序）")
    ap.add_argument("--dump-interfaces", action="store_true",
                    help="启动后打印控制器状态与硬件接口归属（验证「每关节恰好一个属主」）")
    ap.add_argument("--hold-only", type=float, default=SETTINGS["hold_only"],
                    help="不跑姿态，只在起始位形悬停 N 秒并报告漂移/峰值速度（用来量重力前馈是否撑得住）")
    ap.add_argument("--initial-poses", default=",".join(str(v) for v in SIM_INITIAL),
                    help="仿真起始位形（关节1..6，逗号分隔）。默认弯曲位形，避开竖直不稳定点")
    ap.add_argument("--gravity-scale", type=float, default=None,
                    help="启动后用 ros2 param set 设重力前馈系数（0 = 纯 PD，与真机 11-21 的测试口径一致）")
    args = ap.parse_args()
    if args.gravity_varstiff_switch_test and (
        args.mode != "mit" or args.no_launch or args.controllers_file or args.controllers or
        args.split or args.zero_gravity or args.pure_pd or args.kp or args.kd or args.no_safety or
        args.gravity_scale is not None or args.state_machine_test or args.mit_frame_test or
        args.temp_trip_test or args.controller != "b601_mit_arm" or
        args.joints.split(",") != PROJECT["robot"]["arm_joints"]
    ):
        ap.error("--gravity-varstiff-switch-test 使用现行六轴双刚度参数及安全节点")
    if args.state_machine_test and (
        args.mode != "mit" or not args.zero_gravity or args.no_launch or args.controllers_file or
        args.controllers or args.split or args.pure_pd or args.kp or args.kd or
        args.gravity_scale is not None or args.controller != "b601_mit_arm" or
        args.joints.split(",") != PROJECT["robot"]["arm_joints"]
    ):
        ap.error("--state-machine-test 使用现行六轴双刚度参数及零重力夹具")
    if args.mit_frame_test and (
        args.mode != "mit" or not args.zero_gravity or args.no_launch or args.split
    ):
        ap.error("--mit-frame-test 需要由脚本启动零重力 MIT 单实例")

    joints = [j for j in args.joints.split(",") if j]
    project = Path(__file__).resolve().parent.parent
    if not args.controllers_file:
        # 默认用安装好的 MIT 控制器 yaml（不是位置模式那份！否则 controller_manager 里
        # 根本没有这个控制器的 type 声明，spawner 会报 "The 'type' param was not defined"）
        prefix = subprocess.run(
            ["ros2", "pkg", "prefix", "b601_bringup"],
            capture_output=True, text=True, check=False,
        ).stdout.strip()
        filename = (
            "b601_mit_controllers_gravity.yaml"
            if args.gravity_varstiff_switch_test or args.state_machine_test
            else "b601_mit_controllers.yaml"
        )
        args.controllers_file = f"{prefix}/share/b601_bringup/config/{filename}"
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
            "dm_command_mode:=mit_impedance",
            f"controllers:=joint_state_broadcaster {args.controller}",
        ]
        if args.gravity_varstiff_switch_test:
            cmds = [c for c in cmds if not c.startswith("controllers:=")]
            extra_controllers = ["b601_mit_gripper"]
        elif args.controllers:
            cmds = [c for c in cmds if not c.startswith("controllers:=")]
            cmds.append(f"controllers:=joint_state_broadcaster {args.controllers}")

        if args.mode == "pos_vel":
            # 位置模式：controller_manager 用生成的位置模式 yaml，命令接口是 position
            cmds = [c for c in cmds if not c.startswith(
                ("controllers_file:=", "controllers:=", "dm_command_mode:="))]
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
            per_joint_keys = MIT_ARRAY_KEYS
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
                    rp["task_torque_timeout_sec"] = 0.0
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
        if args.temp_trip_test:
            import yaml as _yaml
            truth_path = subprocess.run(
                ["ros2", "pkg", "prefix", "b601_description"],
                capture_output=True, text=True, check=False,
            ).stdout.strip() + "/share/b601_description/config/b601_project.yaml"
            cfg = _yaml.safe_load(Path(truth_path).read_text(encoding="utf-8"))
            hw = cfg["hardware"]["dm"]
            hw["temperature"]["limit_c"] = args.temp_limit
            hw["temperature"]["warn_c"] = args.temp_limit - SETTINGS["thermal_fixture"]["warning_offset_c"]
            hw["thermal"]["ambient_c"] = args.temp_limit - SETTINGS["thermal_fixture"]["ambient_offset_c"]
            hw["thermal"]["heat_c_per_s"] = SETTINGS["thermal_fixture"]["heat_c_per_s"]  # 满载 6 ℃/s：几秒内就能到线
            patched = project / ".verify" / f"truth_temp_{args.tag}.yaml"
            patched.write_text(_yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True),
                               encoding="utf-8")
            cmds.append(f"project_config_file:={patched}")
        if args.joint_friction > 0.0 or args.zero_gravity or args.gravity_varstiff_switch_test:
            from ament_index_python.packages import get_package_share_directory

            source = Path(get_package_share_directory("b601_mujoco")) / "models" / "b601_arm.xml"
            model = ET.parse(source)
            if args.joint_friction > 0.0:
                joints = [joint for joint in model.findall(".//joint")
                          if joint.get("type") in ("hinge", "slide")]
                if not joints:
                    raise RuntimeError("MJCF 缺少可以注入摩擦的关节")
                for joint in joints:
                    joint.set("frictionloss", str(args.joint_friction))
                print(f"   已注入关节库仑摩擦 frictionloss={args.joint_friction}（{len(joints)} 个关节）")
            if args.zero_gravity or not args.gravity_varstiff_switch_test:
                option = model.find("option")
                if option is None:
                    raise RuntimeError(f"{source} 缺少 MuJoCo option")
                option.set("gravity", "0 0 0")
                flag = option.find("flag")
                if flag is None:
                    flag = ET.SubElement(option, "flag")
                flag.set("contact", "disable")
            if args.zero_gravity and args.gravity_scale is None:
                args.gravity_scale = 0.0
            if args.gravity_varstiff_switch_test:
                # 保持试验中两指机械联动，使 MuJoCo 在积分时计算从动指的反力。
                # 后端步末的运动学镜像只保证外观，无法给自由滑动的从动指提供反力。
                equality = model.find("equality")
                if equality is None:
                    equality = ET.SubElement(model.getroot(), "equality")
                robot = PROJECT["robot"]
                # 2 ms 时间常数对应模型的两个物理步长，近刚性的联动需要接近 1 的阻抗。
                ET.SubElement(equality, "joint", joint1=robot["mimic_joint"],
                              joint2=robot["gripper_joint"], polycoef="0 -1 0 0 0",
                              solref="0.002 1", solimp="0.999 0.9999 0.0001")
            fixture = project / ".verify" / f"b601_fixture_{args.tag}.xml"
            model.write(fixture, encoding="unicode")
            cmds.append(f"mujoco_model:={fixture}")
        if args.initial_poses:
            import yaml as _yaml
            vals = [float(v) for v in args.initial_poses.split(",") if v]
            if args.gravity_varstiff_switch_test and len(vals) == len(joints):
                opening = SETTINGS["gravity_fixture_finger_opening_m"]
                vals.extend((opening, -opening))
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
        if args.mit_frame_test or args.state_machine_test or args.gravity_varstiff_switch_test:
            import yaml as _yaml
            config = _yaml.safe_load(Path(args.controllers_file).read_text(encoding="utf-8"))
            params = config[args.controller]["ros__parameters"]
            if args.state_machine_test:
                params["use_gravity"] = False
            if args.gravity_varstiff_switch_test:
                params["gravity_params"] = fixture_gravity_params(source)
            if args.mit_frame_test:
                fixture = SETTINGS["mit_frame"]
                params.update(gravity_scale=fixture["gravity_scale"], use_gravity=True,
                              friction_scale=fixture["friction_scale"],
                              task_torque_timeout_sec=fixture["task_torque_timeout_sec"])
                params["friction"] = [0.0] * len(params["joints"])
                params["friction"][params["joints"].index("joint2")] = fixture["friction_test_nm"]
                args.gravity_scale = None
            test_config = project / ".verify" / f"mit_test_{args.tag}.yaml"
            test_config.write_text(_yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
            args.controllers_file = str(test_config)
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
        deadline = time.monotonic() + SETTINGS["shutdown_timeout_sec"]
        while time.monotonic() < deadline and proc.poll() is None:
            time.sleep(SETTINGS["shutdown_poll_sec"])
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

    rclpy.init()
    node = Harness(args.controller, joints)
    arm_client = node.arm_action_client("b601_arm_controller") if args.mode == "pos_vel" else None
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
                time.sleep(SETTINGS["restart_wait_sec"])
                proc = _spawn()
                ok_up = node_wait_controllers(
                    node, ["joint_state_broadcaster"] + ctrl_names, args.startup_timeout)
            if not ok_up:
                print(f"❌ 控制器没能 active（{args.startup_timeout:.0f}s 超时）；日志：{launch_log}")
                tail = launch_log.read_text(encoding="utf-8", errors="replace").splitlines()[-25:]
                for line in tail:
                    print("   " + line)
                return 1
        if not node.wait_ready(SETTINGS["state_ready_timeout_sec"]):
            print("❌ 收不到 /joint_states")
            return 1
        if args.gravity_scale is not None:
            out = subprocess.run(
                ["ros2", "param", "set", f"/{args.controller}", "gravity_scale",
                 f"{args.gravity_scale}"],
                capture_output=True, text=True, timeout=SETTINGS["parameter_process_timeout_sec"], check=False,
            )
            print(f"gravity_scale := {args.gravity_scale}（启动前已写入控制器 yaml；"
                  f"此处再 set 一次做确认：{out.stdout.strip() or out.stderr.strip()}）")
            node.spin_for(0.5)

        if args.dump_interfaces:
            for cmd in (["ros2", "control", "list_controllers"],
                        ["ros2", "control", "list_hardware_interfaces"]):
                out = subprocess.run(cmd, capture_output=True, text=True, timeout=SETTINGS["interfaces_process_timeout_sec"], check=False).stdout
                print(f"--- {' '.join(cmd[2:])} ---")
                for line in out.splitlines():
                    if "effort" in line or "controller" in line.lower() or "active" in line:
                        print("   " + line.strip())
        if args.mode == "mit":
            node.targets.select(args.startup_timeout)
        if args.mit_frame_test:
            return mit_frame_test(node)
        if args.gravity_varstiff_switch_test:
            return gravity_varstiff_switch_test(node, csv_path)
        if args.temp_trip_test:
            print(f"过温测试：失能线 {args.temp_limit:.0f} ℃，热模型满载 6 ℃/s（需真实重力负载）")
            node.spin_for(1.0)
            hold = node.q()
            end = time.monotonic() + SETTINGS["thermal_fixture"]["observe_sec"]
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
        node.targets.close(SETTINGS["state_ready_timeout_sec"])
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        cleanup()
    return rc


if __name__ == "__main__":
    sys.exit(main())
