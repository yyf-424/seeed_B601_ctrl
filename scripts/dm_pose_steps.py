#!/usr/bin/env python3
"""dm_pose_steps.py —— 真机"多姿态定位精度"测试：逐个姿态下发 → 稳态误差判定

用法（前提：另开终端已 `hardware:=dm dm_enable_motors:=true` 起了 bringup）：

    # 先看计划（不发任何指令）
    ./scripts/dm_pose_steps.py --poses "joint2=-0.9,joint3=-1.1;..." --dry-run

    # 真跑：每个姿态到位后等 1.5 s，再采样 1 s 算稳态误差
    ./scripts/dm_pose_steps.py --poses "joint2=-0.9,joint3=-1.1;joint1=0.4,joint5=0.3"

判据（默认，可用 --err-ratio 改）：
    每个关节的**稳态误差** ≤ 关节行程（max_pos − min_pos）的 **5%**。
    之所以按"行程百分比"而不是固定 rad：六个关节行程差 6 倍（j5 2.3 rad vs j6 6.28 rad），
    固定阈值对小行程关节过松、对大行程关节过严。

安全：
    · 目标必须先过 `trajectory_utils.check_limits`（越界直接拒发，不截断）；
    · 单关节位移不得超过 `--max-delta`（默认 0.6 rad，防手误写错一位数）；
    · 轨迹按 `--slew`（默认 0.15 rad/s）算时长，绝不做阶跃；
    · 全程盯 `/b601_safety/status`：EMERGENCY_STOP 立刻停止后续姿态。
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from b601_demos import trajectory_utils as tu

STATUS_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


def parse_poses(spec: str) -> list[dict[str, float]]:
    """'a=1,b=2;c=3' → [{'a':1.0,'b':2.0}, {'c':3.0}]"""
    out: list[dict[str, float]] = []
    for block in spec.split(";"):
        block = block.strip()
        if not block:
            continue
        target: dict[str, float] = {}
        for part in block.replace("；", ",").split(","):
            part = part.strip()
            if not part:
                continue
            if "=" not in part:
                raise ValueError(f"无法解析 '{part}'（应为 joint2=-0.9）")
            k, v = part.split("=", 1)
            k = k.strip()
            if k not in tu.ARM_JOINTS:
                raise ValueError(f"未知关节 '{k}'")
            target[k] = float(v)
        if target:
            out.append(target)
    return out


class PoseSteps(Node):
    def __init__(self, controller: str):
        super().__init__("b601_pose_steps")
        self.current: dict[str, float] = {}
        self.current_vel: dict[str, float] = {}
        self.safety = "UNKNOWN"
        self.create_subscription(JointState, "/joint_states", self._on_state, 50)
        self.create_subscription(DiagnosticStatus, "/b601_safety/status", self._on_status, STATUS_QOS)
        self.client = ActionClient(
            self, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory")
        self._samples: list[dict[str, float]] = []
        self._collect = False

    def _on_state(self, msg: JointState) -> None:
        values = dict(zip(msg.name, msg.position))
        if msg.velocity and len(msg.velocity) == len(msg.name):
            self.current_vel = dict(zip(msg.name, msg.velocity))
        if all(n in values for n in tu.ARM_JOINTS):
            self.current = values
            if self._collect:
                self._samples.append(dict(values))

    def _on_status(self, msg: DiagnosticStatus) -> None:
        for kv in msg.values:
            if kv.key == "state":
                self.safety = kv.value

    def wait_ready(self, timeout: float = 15.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current and self.safety != "UNKNOWN":
                return True
        return False

    def range_of(self, name: str) -> float:
        """关节行程（max_pos − min_pos），从统一真值读。"""
        cfg = tu._load_project_config()
        j = cfg["joints"][name]
        return float(j["max_pos"]) - float(j["min_pos"])

    def wait_settled(self, timeout: float, pos_tol: float = 5e-4, need: float = 0.4) -> bool:
        """等到所有关节位置连续 need 秒不再变化 —— 这才是"稳态"。

        为什么要主动判：JTC 的 goal 在轨迹时间结束时就算完成，但机械臂还会再走
        零点几秒才真停住。固定 sleep 一会就采样，会把"还在收敛"当成"稳态误差"，
        2026-11-21 第一次跑 5 姿态就因此报了 8–11% 的假超标（settle 1.5 s 太短）。
        为什么不用速度判：达妙电机静止时速度反馈本身有 ±0.04 rad/s 的抖动量，
        阈值设 0.02 会永远"没停稳"（2026-11-21 实测 5 个姿态全部超时告警）。
        改用**位置变化量**判：0.4 s 内所有关节位移都 < 5e-4 rad 才算稳。
        """
        t0 = time.monotonic()
        good = 0.0
        ref = dict(self.current)
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            if not self.current:
                continue
            moved = max(abs(self.current[n] - ref[n]) for n in tu.ARM_JOINTS)
            if moved < pos_tol:
                good += 0.02
            else:
                good = 0.0
                ref = dict(self.current)
            if good >= need:
                return True
        return False

    def spin_for(self, seconds: float) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.02)

    def run_pose(self, index: int, total: int, target: dict[str, float],
                 slew: float, settle: float, sample_sec: float,
                 max_delta: float, preview_start: dict[str, float] | None) -> tuple[bool, dict[str, tuple]]:
        start = {n: self.current[n] for n in tu.ARM_JOINTS} if preview_start is None else preview_start
        want = {n: target.get(n, start[n]) for n in tu.ARM_JOINTS}

        problems = tu.check_limits(tu.ARM_JOINTS, [want[n] for n in tu.ARM_JOINTS])
        if problems:
            for p in problems:
                self.get_logger().error(f"[{index}/{total}] 拒发：{p}")
            return False, {}
        over = [n for n in tu.ARM_JOINTS if abs(want[n] - start[n]) > max_delta + 1e-9]
        if over:
            self.get_logger().error(
                f"[{index}/{total}] 拒发：{', '.join(over)} 位移超过 --max-delta {max_delta} rad")
            return False, {}

        move = max(abs(want[n] - start[n]) for n in tu.ARM_JOINTS)
        dur = max(move / max(slew, 1e-3), 1.5)
        self.get_logger().info(
            f"[{index}/{total}] " + ", ".join(f"{n}={want[n]:+.3f}" for n in tu.ARM_JOINTS)
            + f"  最大位移 {move:.3f} rad，时长 {dur:.1f}s（{slew:.2f} rad/s）")

        if preview_start is not None:
            preview_start.update(want)
            return True, {}

        traj = tu.make_trajectory(
            tu.ARM_JOINTS,
            [[start[n] for n in tu.ARM_JOINTS], [want[n] for n in tu.ARM_JOINTS]],
            [0.0, dur])
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = tu.duration(2.0)

        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"[{index}/{total}] 等不到 action server")
            return False, {}
        # ⚠ 必须等 **action 结果**，不能只看 goal 被接受：
        #   send_goal_with_retry 返回的只是 goal_handle；不等结果就继续发下一个目标，
        #   JTC 会用新 goal **抢占**当前轨迹（2026-11-21 实测：pose 2 只走到 70% 就被
        #   pose 3 抢掉，报出 8–15% 的假超标）。之前"全过"其实是运气：那时等停稳固定
        #   耗 6 s，恰好比轨迹 4 s 长。
        handle = tu.send_goal_with_retry(
            self, self.client, goal, f"pose {index}/{total}", timeout_sec=dur + 40.0)
        if handle is None or not handle.accepted:
            self.get_logger().error(f"[{index}/{total}] goal 未接受")
            return False, {}
        result = tu.spin_until(
            self, handle.get_result_async(), timeout_sec=dur + 40.0,
            what=f"pose {index}/{total} 结果")
        if result is None:
            self.get_logger().error(f"[{index}/{total}] 等不到 action 结果（轨迹可能仍在跑）")
            return False, {}
        code = result.result.error_code
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"[{index}/{total}] 轨迹失败 error_code={code} "
                f"error_string={result.result.error_string!r}")
            return False, {}
        if self.safety == "EMERGENCY_STOP":
            self.get_logger().error(f"[{index}/{total}] 执行期间触发急停，停止后续姿态")
            return False, {}

        # 稳态判定：先确认真的停稳（速度连续低于阈值），再采样 sample_sec
        if not self.wait_settled(timeout=max(settle, 6.0)):
            self.get_logger().warn(f"[{index}/{total}] 等停稳超时，按当前状态采样（结果可能偏大）")
        self.spin_for(settle * 0.5)
        self._samples = []
        self._collect = True
        self.spin_for(sample_sec)
        self._collect = False
        if not self._samples:
            self.get_logger().error(f"[{index}/{total}] 采样期间没收到 /joint_states")
            return False, {}

        stats: dict[str, tuple] = {}
        all_ok = True
        for n in tu.ARM_JOINTS:
            vals = [s[n] for s in self._samples]
            mean = sum(vals) / len(vals)
            worst = max(abs(v - mean) for v in vals)
            err = abs(mean - want[n])
            rng = self.range_of(n) or 1.0
            pct = err / rng * 100.0
            limit_pct = 5.0
            stats[n] = (want[n], mean, err, pct, worst)
            if pct > limit_pct:
                all_ok = False
        return all_ok, stats


def main() -> int:
    ap = argparse.ArgumentParser(description="真机多姿态定位精度测试（稳态误差判据）")
    ap.add_argument("--poses", required=True, help="姿态用 ; 分隔，例：joint2=-0.9;joint1=0.3,joint5=0.2")
    ap.add_argument("--controller", default="b601_arm_controller")
    ap.add_argument("--slew", type=float, default=0.15, help="轨迹平均速率 rad/s（默认 0.15，很慢）")
    ap.add_argument("--settle", type=float, default=1.5, help="到位后等待稳定时间 s")
    ap.add_argument("--sample-sec", type=float, default=1.0, help="稳态采样时长 s")
    ap.add_argument("--max-delta", type=float, default=0.6, help="单关节允许的最大位移 rad")
    ap.add_argument("--dry-run", action="store_true", help="只打印计划，不发指令")
    args = ap.parse_args()

    try:
        poses = parse_poses(args.poses)
    except ValueError as e:
        print(f"❌ {e}", file=sys.stderr)
        return 2
    if not poses:
        print("❌ 没有解析到任何姿态", file=sys.stderr)
        return 2

    rclpy.init()
    node = PoseSteps(args.controller)
    failures = 0
    try:
        if not node.wait_ready():
            print("❌ 等不到 /joint_states 或 /b601_safety/status（bringup 起来了吗）", file=sys.stderr)
            return 1
        print("起始位形：" + ", ".join(f"{n}={node.current[n]:+.3f}" for n in tu.ARM_JOINTS))
        print(f"安全状态 {node.safety}；判据：每关节稳态误差 ≤ 行程的 {5.0:.0f}%")

        preview_start = {n: node.current[n] for n in tu.ARM_JOINTS} if args.dry_run else None
        for i, target in enumerate(poses, 1):
            ok, stats = node.run_pose(
                i, len(poses), target, args.slew, args.settle, args.sample_sec,
                args.max_delta, preview_start)
            if args.dry_run:
                if not ok:
                    failures += 1
                    break
                continue
            if not stats:
                failures += 1
                print(f"[{i}/{len(poses)}] ❌ 未完成")
                continue
            print(f"\n--- 姿态 {i}/{len(poses)} 稳态误差 ---")
            print("关节      目标      实测均值   误差(rad)  占行程%  采样抖动")
            for n in tu.ARM_JOINTS:
                want, mean, err, pct, worst = stats[n]
                mark = "✅" if pct <= 5.0 else "❌"
                print(f"{mark}{n:9s} {want:+8.4f} {mean:+9.4f} {err:9.4f} {pct:8.2f} {worst:9.4f}")
            if not ok:
                failures += 1
                print(f"❌ 姿态 {i} 有超标关节（>5% 行程）")
            if node.safety == "EMERGENCY_STOP":
                print("🛑 已急停，停止后续姿态")
                break

        if args.dry_run:
            print(f"\n（dry-run）计划 {len(poses)} 个姿态，全部通过限位/位移校验：{failures == 0}")
        else:
            verdict = "全部通过" if failures == 0 else f"{failures} 个姿态超标"
            print(f"\n汇总：{verdict}（判据 5% 行程）")
        return 0 if failures == 0 else 1
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n已中止")
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
