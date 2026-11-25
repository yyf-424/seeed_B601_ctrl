#!/usr/bin/env python3
"""test_safety.py —— 安全判定的离线单测（不需要 ROS，不需要起节点）。

    ./scripts/test_safety.py            # 跑判定矩阵
    ./scripts/test_safety.py -v         # 打印每个用例的判定理由

被测对象：ros2_ws/src/b601_safety/b601_safety/logic.py
阈值来源：ros2_ws/src/b601_safety/config/safety_params.yaml（由 b601_project.yaml 生成）
          —— 所以改了统一配置之后，这个测试会自动按新阈值判定。

为什么要有这个测试：
    "安全节点会不会误停 / 会不会该停不停" 是上真机前必须回答的问题，
    而这个问题完全可以离线验证：喂一帧假数据，看判定结果对不对。
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import importlib.util
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
from project_config import load_project_config

SETTINGS = load_project_config()["tools"]["test_safety"]
LOGIC = REPO_ROOT / "ros2_ws" / "src" / "b601_safety" / "b601_safety" / "logic.py"
PARAMS = REPO_ROOT / "ros2_ws" / "src" / "b601_safety" / "config" / "safety_params.yaml"


def load_logic():
    spec = importlib.util.spec_from_file_location("b601_safety_logic", LOGIC)
    module = importlib.util.module_from_spec(spec)
    # dataclasses 解析注解时要去 sys.modules 找自己的模块，必须先登记再 exec
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def load_limits(logic, params_path: Path):
    import yaml

    with params_path.open(encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    params = next(iter(cfg.values()))["ros__parameters"]
    return logic.SafetyLimits(
        names=list(params["joint_names"]),
        pos_min=[float(v) for v in params["pos_min"]],
        pos_max=[float(v) for v in params["pos_max"]],
        vel_max=[float(v) for v in params["vel_max"]],
        startup=[float(v) for v in params["startup_pose"]],
        warn_pos_margin=[float(v) for v in params["warn_pos_margin"]],
        warn_near_limit=[bool(v) for v in params["warn_near_limit"]],
        warn_pose_deviation_joint=[bool(v) for v in params["warn_pose_deviation_joint"]],
        warn_vel_ratio=float(params["warn_vel_ratio"]),
        warn_on_pose_deviation=bool(params["warn_on_pose_deviation"]),
        warn_pose_threshold=float(params["warn_pose_threshold"]),
    ), params


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logic = load_logic()
    limits, params = load_limits(logic, PARAMS)

    startup = dict(zip(limits.names, limits.startup))
    still = {n: 0.0 for n in limits.names}
    cases = []

    def case(name, expect, positions, velocities=None, note=""):
        cases.append((name, expect, positions, velocities or still, note))

    # 1) 就位在启动姿态，速度为 0 → 正常
    case("启动姿态静止", logic.NORMAL, dict(startup))

    # 2) 在限位内但明显离开启动位形 → 警告（不是急停）
    p = dict(startup); p["joint2"] = startup["joint2"] - SETTINGS["pose_deviation_rad"]
    case("离开启动位形 0.6rad", logic.WARNING, p, note="警告态必须允许继续驱动")

    # 3) 接近 joint2 急停下限 → 警告
    p = dict(startup, joint2=limits.pos_min[1] + limits.warn_pos_margin[1] / 2)
    case("接近 J2 下限", logic.WARNING, p)

    # 4) 角速度达到上限的 85% → 警告
    v = dict(still); v["joint1"] = SETTINGS["warning_speed_ratio"] * limits.vel_max[0]
    case("角速度 85% 上限", logic.WARNING, dict(startup), v)

    # 5) 角速度越限 → 急停
    v = dict(still); v["joint1"] = SETTINGS["estop_speed_ratio"] * limits.vel_max[0]
    case("角速度 105% 上限", logic.EMERGENCY_STOP, dict(startup), v)

    # 6) 角度越限 → 急停
    p = dict(startup); p["joint1"] = limits.pos_max[0] + SETTINGS["arm_position_overrun_rad"]
    case("角度越上限 0.10rad", logic.EMERGENCY_STOP, p)

    # 7) 夹爪越限（m 单位）→ 急停
    p = dict(startup); p["finger_left"] = limits.pos_max[-2] + SETTINGS["gripper_overrun_m"]
    case("夹爪越限", logic.EMERGENCY_STOP, p)

    # 8) 缺关节反馈 → 警告
    p = dict(startup); p.pop("joint6")
    case("缺 joint6 反馈", logic.WARNING, p)

    # 9) 角度与速度同时越限 → 仍是急停，且理由两条都在
    p = dict(startup); p["joint3"] = limits.pos_min[2] - SETTINGS["arm_position_overrun_rad"]
    v = dict(still); v["joint4"] = SETTINGS["dual_fault_speed_ratio"] * limits.vel_max[3]
    case("角度+速度同时越限", logic.EMERGENCY_STOP, p, v, note="两条理由都要报出来")

    # 10) 速度刚好在警告阈值下 → 正常（阈值边界不应抖动）
    p = dict(startup)
    v = dict(still); v["joint1"] = SETTINGS["normal_speed_ratio"] * limits.vel_max[0]
    case("角速度 50% 上限", logic.NORMAL, p, v)

    # 12) 关节正好在限位上（含噪声）→ 警告（贴限位），但绝不能判越限急停
    p = dict(startup); p["joint2"] = limits.pos_max[1] - SETTINGS["limit_encoder_noise_rad"]
    case("关节贴限位边界", logic.WARNING, p, note="只警告，不能因 1e-6 噪声判越限")

    failures = 0
    print(f"阈值来源：{PARAMS.relative_to(REPO_ROOT)}")
    print(f"  warn_pos_margin={limits.warn_pos_margin}")
    print(f"  warn_near_limit={limits.warn_near_limit}"
          f"  warn_pose_deviation_joint={limits.warn_pose_deviation_joint}")
    print(f"  warn_pose_threshold={limits.warn_pose_threshold}"
          f"（全局启用={limits.warn_on_pose_deviation}）\n")

    for name, expect, positions, velocities, note in cases:
        result = logic.evaluate(positions, velocities, limits)
        ok = result.state == expect
        failures += 0 if ok else 1
        mark = "\033[1;32m[PASS]\033[0m" if ok else "\033[1;31m[FAIL]\033[0m"
        print(f"{mark} {name:<22} 期望 {expect:<15} 实际 {result.state:<15}"
              + (f"  ← {note}" if note else ""))
        if args.verbose or not ok:
            for reason in result.reasons:
                print(f"         · {reason}")

    profile_cases = []
    if "low_vel_max" in params:
        low = replace(limits, vel_max=params["low_vel_max"], stop_at_velocity_limit=frozenset(limits.names))
        finger = limits.names.index("finger_left")
        normal_speed = limits.vel_max[finger]
        low_speed = low.vel_max[finger]
        profile_cases += [
            ("低档速度到达 10 rad/s 急停", low, startup, dict(still, joint1=low.vel_max[0]), True),
            ("低档速度在扩大范围内", low, startup,
             dict(still, joint1=(low.vel_max[0] + limits.vel_max[0]) / 2), False),
            ("夹爪常规速度等于上限", limits, startup, dict(still, finger_left=normal_speed), False),
            ("夹爪常规速度超过上限急停", limits, startup,
             dict(still, finger_left=math.nextafter(normal_speed, math.inf)), True),
            ("夹爪低档速度低于上限", low, startup,
             dict(still, finger_left=math.nextafter(low_speed, 0.0)), False),
            ("夹爪低档速度到达上限急停", low, startup, dict(still, finger_left=low_speed), True),
        ]
        for label, profile in [("常规档", limits), ("低档", low)]:
            for joint, lower, upper in zip(limits.names, limits.pos_min, limits.pos_max):
                for bound, outside in [(lower, -math.inf), (upper, math.inf)]:
                    profile_cases += [
                        (f"{label} {joint} 界内", profile, dict(startup, **{joint: bound}), still, False),
                        (f"{label} {joint} 越界", profile,
                         dict(startup, **{joint: math.nextafter(bound, outside)}), still, True),
                    ]
    for name, profile, positions, velocities, should_stop in profile_cases:
        result = logic.evaluate(positions, velocities, profile)
        ok = (result.state == logic.EMERGENCY_STOP) == should_stop
        failures += not ok
        print(f"{'[PASS]' if ok else '[FAIL]'} {name:<22} 实际 {result.state}")

    total = len(cases) + len(profile_cases)
    print(f"\n汇总：{total - failures}/{total} 通过")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
