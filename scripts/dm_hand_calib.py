#!/usr/bin/env python3
"""dm_hand_calib.py —— 手推标定：确认「电机序号 ↔ 物理关节」映射与正方向

为什么需要这一步：
  URDF 里的关节正方向是建模选择，电机编码器的正方向是硬件事实，两者可能相反。
  在不开电机（不使能）的情况下，让手臂被手推动，看**哪个电机读数变化、往哪变**，
  就能一次性确定每个关节的 direction（+1/-1），以及有没有装反/换过的关节。

用法（电机必须处于未使能状态 —— 探针里 status_code=0 就是未使能）：
    ./scripts/dm_hand_calib.py                # 含 10 秒准备时间，之后每段 5 秒
    ./scripts/dm_hand_calib.py --lead 5 --phase 4

运行后会提示按顺序做 7 个动作，结束后打印一张表：
    每一段是哪个电机的读数变了、变化量是多少（正/负）
据此写回 b601_description/urdf/inc/ros2_control.xacro 的 <param name="direction">。

安全：全程只读，不发使能、不发运动命令。但**未使能的手臂会被重力拽动**，
      请先托住大臂再开始，动作要慢，别硬推到机械限位。
"""

import argparse
import sys
import time

# (阶段说明, 对应关节名建议)
PHASES = [
    ("别动 —— 取基准", None),
    ("转 底座（整条手臂左右转，最下面那个关节）", "joint1"),
    ("抬/压 大臂（从底座数第二个关节）", "joint2"),
    ("弯/伸 小臂（肘部那个关节）", "joint3"),
    ("扭 腕部关节 A（手腕第一个转轴）", "joint4"),
    ("摆 腕部关节 B（手腕第二个转轴）", "joint5"),
    ("转 末端法兰（夹爪整体的自转）", "joint6"),
    ("开/合 夹爪手指", "gripper"),
]

MOTORS = [(1, 0x11, "4340P", "joint1"), (2, 0x12, "4340P", "joint2"), (3, 0x13, "4340P", "joint3"),
          (4, 0x14, "4310", "joint4"), (5, 0x15, "4310", "joint5"), (6, 0x16, "4310", "joint6"),
          (7, 0x17, "4310", "gripper")]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--lead", type=float, default=10.0, help="准备时间（秒）")
    ap.add_argument("--phase", type=float, default=5.0, help="每段时长（秒）")
    ap.add_argument("--rate", type=float, default=20.0, help="采样频率 Hz")
    args = ap.parse_args()

    from motorbridge import Controller

    ctrl = Controller.from_dm_serial(args.device, args.baud)
    motors = [(mid, name, ctrl.add_damiao_motor(mid, fid, model)) for mid, fid, model, name in MOTORS]

    def sample():
        for _, _, m in motors:
            try:
                m.request_feedback()
            except Exception:  # noqa: BLE001
                pass
        try:
            ctrl.poll_feedback_once()
        except Exception:  # noqa: BLE001
            pass
        out = {}
        for mid, name, m in motors:
            try:
                st = m.get_state()
            except Exception:  # noqa: BLE001
                st = None
            if st is not None:
                out[mid] = float(st.pos)
        return out

    print("=" * 72)
    print("手推标定：全程只读，不使能电机。")
    print("⚠ 未使能的手臂会被重力拽动 —— 先托住大臂，动作慢，别推到限位。")
    print("=" * 72)

    print(f"\n准备：{args.lead:.0f} 秒后开始（先托住手臂）")
    for k in range(int(args.lead), 0, -1):
        print(f"  {k}...", end="\r", flush=True)
        time.sleep(1)
    print(" " * 20, end="\r")

    # 基准
    print(f"\n[0] {PHASES[0][0]}")
    base_samples = []
    t0 = time.time()
    while time.time() - t0 < args.phase:
        base_samples.append(sample())
        time.sleep(1.0 / args.rate)
    base = {mid: sorted(s[mid] for s in base_samples if mid in s)[len(base_samples)//2]
            for mid in (m[0] for m in motors) if any(mid in s for s in base_samples)}
    print("    基准: " + " ".join(f"{mid}:{base.get(mid, float('nan')):+.4f}" for mid, _, _ in motors))

    results = {}   # phase_index -> (avg_delta per motor)
    for idx, (desc, joint_hint) in enumerate(PHASES[1:], start=1):
        print(f"\n[{idx}] {desc}")
        deltas = {mid: [] for mid, _, _ in motors}
        t0 = time.time()
        while time.time() - t0 < args.phase:
            cur = sample()
            for mid, val in cur.items():
                if mid in base:
                    deltas[mid].append(val - base[mid])
            time.sleep(1.0 / args.rate)
        # 取每台电机位移绝对值的最大者作为该段代表值（保持符号）
        summary = {}
        for mid, seq in deltas.items():
            if not seq:
                continue
            peak = max(seq, key=abs)
            summary[mid] = peak
        results[idx] = (summary, joint_hint)
        moved = {mid: v for mid, v in summary.items() if abs(v) > 0.01}
        if moved:
            print("    变化: " + " ".join(f"motor{mid}={v:+.3f}" for mid, v in sorted(moved.items())))
        else:
            print("    变化: （没检测到 >0.01 的运动，这一段可能没推动）")

    # ---------------------------------------------------------------- 汇总
    print("\n" + "=" * 72)
    print("汇总（段号 → 有变化的电机，Δ 为相对基准的峰值位移）")
    print("=" * 72)
    mapping = {}
    for idx, (summary, hint) in results.items():
        moved = {mid: v for mid, v in summary.items() if abs(v) > 0.01}
        line = " ".join(f"motor{mid}{v:+.3f}" for mid, v in sorted(moved.items())) or "（无）"
        print(f"  段{idx} 建议是 {hint:<8} → {line}")
        if moved:
            mid = max(moved, key=lambda k: abs(moved[k]))
            mapping[hint] = (mid, moved[mid])

    print("\n解读方法：")
    print("  1) 若 段N 只有 motorN 动 → 序号映射正常（motor1..6 ↔ joint1..6，motor7 ↔ 夹爪）")
    print("  2) 若某段动了别的电机 → 对应关系要改 ros2_control.xacro 的 motor_id")
    print("  3) 方向：把「该关节在 URDF 里增大时的物理方向」跟上表的 Δ 符号对照；")
    print("     Δ 为正而 URDF 增大方向相反时，把该关节 direction 设为 -1")
    print("  4) 数值 ≈ 电机 rad；夹爪需要再乘 scale（当前 0.01 m/rad）才是米")

    ctrl.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
