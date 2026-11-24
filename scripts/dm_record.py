#!/usr/bin/env python3
"""dm_record.py —— 录制关节状态（位置 / **速度** / 力矩）与安全状态到 CSV。

    ./scripts/dm_record.py                        # 录 10 秒到 .verify/run_<时间戳>.csv
    ./scripts/dm_record.py --duration 30          # 录 30 秒
    ./scripts/dm_record.py --out .verify/a.csv    # 指定输出
    ./scripts/dm_record.py --duration 0           # 一直录到 Ctrl-C

为什么需要它（2026-11-21 的教训）：
    2026-11-20 真机"抽风"时**没有任何速度数据留存**，事后只能靠"joint6 转了 175°"这种
    位移描述，峰值角速度不可考、阈值也没法按真实噪声校准。此后规定：任何真机动作
    （尤其力控/复测）都必须留下 q̇ 记录。

与插件内 CSV（`dm_log_csv:=/path/run.csv`）的分工：
    · 本脚本走 ROS：录的是**发布出来的** /joint_states（已经过插件合理性过滤），
      好处是真机/仿真/远端都能用，还能同时记录安全状态；教学工作流首选它。
    · 插件内 CSV 录的是**电机原始读数（过滤前）**+ 命令值，排查"抽风"这种
      数据被守卫丢弃的场景必须用它。

输出列：t, <joint>_pos, <joint>_vel, <joint>_effort ...，最后一列 safety_state
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from project_config import load_project_config

PROJECT = load_project_config()
SETTINGS = PROJECT["tools"]["dm_record"]

# 安全状态话题是 transient_local（锁存）：晚加入的订阅者也应立刻拿到"当前状态"。
# 用默认 volatile 订阅会收不到（状态不变时发布者不会重复发）—— 2026-11-21 实测踩过。
STATUS_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


REPO_ROOT = Path(__file__).resolve().parent.parent
SAFETY_PARAMS = REPO_ROOT / "ros2_ws" / "src" / "b601_safety" / "config" / "safety_params.yaml"


def load_limits():
    """读安全参数，拿到"急停/警告"速度线，用于事后统计超阈值次数。"""
    try:
        import yaml

        with SAFETY_PARAMS.open(encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        params = next(iter(cfg.values()))["ros__parameters"]
        return (
            list(params["joint_names"]),
            [float(v) for v in params["vel_max"]],
            float(params["estop_vel_ratio"]),
            float(params["warn_vel_ratio"]),
        )
    except Exception:  # noqa: BLE001 —— 没有配置就不做超阈值统计，不影响录制
        velocity = PROJECT["safety"]["velocity"]
        return None, None, velocity["estop_ratio"], velocity["warn_ratio"]


class Recorder(Node):
    def __init__(self, out_path: Path, joints: list[str] | None):
        super().__init__("b601_record")
        self.out_path = out_path
        self.rows: list[list[float]] = []
        self.names: list[str] = []
        self.wanted = joints
        self.safety_state = ""
        self.t0: float | None = None
        self.latest: JointState | None = None

        self.create_subscription(JointState, "/joint_states", self._on_state, 100)
        self.create_subscription(
            DiagnosticStatus, "/b601_safety/status", self._on_status, STATUS_QOS
        )
        self.create_timer(0.1, self._tick)   # 10 Hz 采样落表（/joint_states 本身 200 Hz）
        self.get_logger().info(f"开始录制 → {self.out_path}")

    def _on_state(self, msg: JointState) -> None:
        self.latest = msg
        if not self.names:
            names = list(msg.name)
            if self.wanted:
                names = [n for n in names if n in self.wanted]
            self.names = names

    def _on_status(self, msg: DiagnosticStatus) -> None:
        for kv in msg.values:
            if kv.key == "state":
                self.safety_state = kv.value

    def _tick(self) -> None:
        msg = self.latest
        if msg is None or not self.names:
            return
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        values = dict(zip(msg.name, msg.position))
        vels = dict(zip(msg.name, msg.velocity)) if len(msg.velocity) == len(msg.name) else {}
        efforts = dict(zip(msg.name, msg.effort)) if len(msg.effort) == len(msg.name) else {}
        row = [now - self.t0]
        for name in self.names:
            row.append(values.get(name, float("nan")))
            row.append(vels.get(name, float("nan")))
            row.append(efforts.get(name, float("nan")))
        self.rows.append(row)
        self.latest = None      # 只在本周期用一次，保证采样点互不重复

    def write_csv(self) -> None:
        header = ["t"]
        for name in self.names:
            header += [f"{name}_pos", f"{name}_vel", f"{name}_effort"]
        header.append("safety_state")
        with self.out_path.open("w", encoding="utf-8") as f:
            f.write("# b601_record：/joint_states 采样（位置/速度/力矩）+ 安全状态\n")
            f.write(",".join(header) + "\n")
            for row in self.rows:
                f.write(",".join(f"{v:.6f}" if isinstance(v, float) else str(v) for v in row))
                f.write("," + self.safety_state + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="录制 /joint_states（含速度）到 CSV")
    parser.add_argument("--duration", type=float, default=SETTINGS["duration"], help="秒；0 = 直到 Ctrl-C")
    parser.add_argument("--out", default="", help="输出 CSV（默认 .verify/run_<时间戳>.csv）")
    parser.add_argument("--joints", default="", help="只记录这些关节（逗号分隔，默认全部）")
    args = parser.parse_args()

    out = Path(args.out) if args.out else (
        REPO_ROOT / ".verify" / f"run_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    wanted = [j.strip() for j in args.joints.split(",") if j.strip()] or None

    rclpy.init()
    node = Recorder(out, wanted)
    t_end = time.monotonic() + args.duration if args.duration > 0 else None
    try:
        while rclpy.ok() and (t_end is None or time.monotonic() < t_end):
            rclpy.spin_once(node, timeout_sec=SETTINGS["poll_sec"])
    except KeyboardInterrupt:
        pass
    finally:
        node.write_csv()
        node.destroy_node()
        rclpy.try_shutdown()

    # ------------------------------------------------------------------ 摘要
    names, vel_max, estop_ratio, warn_ratio = load_limits()
    n = len(node.rows)
    print(f"\n已写入 {out}（{n} 行 × {len(node.names)} 关节）")
    if n == 0 or not node.rows:
        print("⚠ 没采到数据：先起 bringup（/joint_states 要有人发）")
        return 0
    dur = node.rows[-1][0] - node.rows[0][0]
    print(f"采样时长 {dur:.2f}s；安全状态最后一次 = {node.safety_state or '未知'}\n")
    print(f"{'关节':<12}{'最大|q̇|(rad/s)':>18}{'最大|τ|(N·m)':>14}{'q 范围':>22}")
    for i, name in enumerate(node.names):
        vels = [abs(row[1 + 3 * i + 1]) for row in node.rows]
        effs = [abs(row[1 + 3 * i + 2]) for row in node.rows]
        poss = [row[1 + 3 * i] for row in node.rows]
        vmax = max(v for v in vels if v == v) if any(v == v for v in vels) else float("nan")
        emax = max(e for e in effs if e == e) if any(e == e for e in effs) else float("nan")
        pmin, pmax = min(poss), max(poss)
        print(f"{name:<12}{vmax:>18.3f}{emax:>14.3f}{pmin:>10.3f} … {pmax:>8.3f}")
        if names and name in names and vel_max:
            k = names.index(name)
            estop_line = estop_ratio * vel_max[k]
            warn_line = warn_ratio * vel_max[k]
            over_warn = sum(1 for v in vels if v == v and v >= warn_line)
            over_stop = sum(1 for v in vels if v == v and v > estop_line)
            if over_warn:
                flag = "🛑 超急停线" if over_stop else "⚠ 超警告线"
                print(f"{'':12}  {flag}：警告线 {warn_line:.2f} 命中 {over_warn} 次 / "
                      f"急停线 {estop_line:.2f} 命中 {over_stop} 次（按 10 Hz 采样计）")
    print("\n提示：真机排查请同时用插件内 CSV（过滤前的电机原始值）：")
    print('  ros2 launch b601_bringup bringup.launch.py hardware:=dm dm_log_csv:="$PWD/.verify/raw.csv"')
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
