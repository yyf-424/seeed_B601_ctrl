#!/usr/bin/env python3
"""dm_probe.py —— 达妙电机只读探针（不经过 ROS，不使能电机）

用途：插上机械臂后**第一件事**跑它，确认"能通、ID 对、角度合理"，
再去碰 ros2_control。它只做三件事：打开串口、注册电机、读反馈，默认不发任何命令。

    ./scripts/dm_probe.py                        # 默认读 3 秒
    ./scripts/dm_probe.py --device /dev/ttyACM0 --seconds 5
    ./scripts/dm_probe.py --enable               # 额外做一次"使能→立刻失能"的连通测试

为什么单独写一个脚本而不是直接开 ros2_control：
  - ros2_control 一 activate 就会进入控制状态，失败信息埋在 controller_manager 日志里；
  - 这里出问题只有一个原因（串口 / ID / 供电），排查面小得多；
  - 它用的是官方 Python SDK 的传输层 motorbridge，和 C++ 插件访问的是同一个库，
    所以这里通了，插件那边基本也通。

安全：默认不使能、不发运动命令。加 --enable 时会"使能后立刻失能"，
      手臂可能瞬间绷紧一下（力矩很小），请先固定好夹具、清空工作区。
"""

import argparse
import sys
import time

from project_config import load_project_config

PROJECT = load_project_config()
SETTINGS = PROJECT["tools"]["dm_probe"]
DM = PROJECT["hardware"]["dm"]
ROBOT = PROJECT["robot"]
JOINTS = [*ROBOT["arm_joints"], ROBOT["gripper_joint"]]


def main() -> int:
    parser = argparse.ArgumentParser(description="B601-DM 只读探针")
    parser.add_argument("--device", default=DM["device"])
    parser.add_argument("--baud", type=int, default=DM["baudrate"])
    parser.add_argument("--seconds", type=float, default=SETTINGS["duration_sec"])
    parser.add_argument("--enable", action="store_true",
                        help="做一次使能→立刻失能的连通测试（不适合首次上电）")
    args = parser.parse_args()

    try:
        from motorbridge import Controller
    except ImportError:
        print("❌ 没有 motorbridge：pip install motorbridge", file=sys.stderr)
        return 2

    print(f"==> 打开 {args.device} @ {args.baud}")
    try:
        ctrl = Controller.from_dm_serial(args.device, args.baud)
    except Exception as exc:  # noqa: BLE001 - 只要失败就原样打印
        print(f"❌ 打不开串口：{exc}", file=sys.stderr)
        print("   排查：① 线插好了吗（ls -l /dev/ttyACM*）"
              "② 你在 dialout 组吗（groups | grep dialout）"
              "③ 是否有别的程序占着这个串口", file=sys.stderr)
        return 1

    motors = {}
    try:
        for name in JOINTS:
            motor = PROJECT["joints"][name]["motor"]
            motors[name] = ctrl.add_damiao_motor(motor["id"], motor["feedback_id"], motor["model"])
        print(f"==> 已注册 {len(motors)} 台电机，开始读反馈 {args.seconds:.1f} s\n")

        deadline = time.time() + args.seconds
        latest = {}
        while time.time() < deadline:
            # ⚠ 达妙电机是"应答式"的：不显式发请求帧就永远读不到数据
            #   （实测：裸读串口 1.5 s 收到 0 字节；发请求后 7 台电机立刻有反馈）
            for motor in motors.values():
                try:
                    motor.request_feedback()
                except Exception:  # noqa: BLE001
                    pass
            try:
                ctrl.poll_feedback_once()
            except Exception:  # noqa: BLE001 - 队列空是正常情况
                pass
            for name, motor in motors.items():
                try:
                    state = motor.get_state()
                except Exception:  # noqa: BLE001
                    continue
                if state is not None and getattr(state, "has_value", 1):
                    latest[name] = state
            time.sleep(SETTINGS["poll_sec"])

        if not latest:
            print("❌ 一帧反馈都没收到。逐项检查：")
            print("   1) 24 V 电源是否上电、电机指示灯")
            print("   2) CAN_H/CAN_L 是否接反、总线两端 120 Ω 终端电阻")
            print("   3) feedback_id 是否与电机实际设置一致（出厂 0x11–0x17）")
            print("   4) 是否有另一个程序同时在收发这条总线")
            return 1

        print(f"{'关节':<9}{'motor':>6}{'反馈id':>8}{'位置(rad/m)':>12}{'速度':>10}"
              f"{'力矩':>9}{'MOS温度':>9}  限位检查")
        ok = True
        for name in JOINTS:
            joint = PROJECT["joints"][name]
            motor = joint["motor"]
            mid, fid = motor["id"], motor["feedback_id"]
            lo, hi = joint["min_pos"], joint["max_pos"]
            state = latest.get(name)
            if state is None:
                print(f"{name:<9}{mid:>6}{fid:>8}{'—— 无反馈 ——':>28}")
                ok = False
                continue
            factor = motor["direction"] * motor["scale"]
            pos = float(state.pos) * factor + motor["zero_offset"]
            velocity = float(state.vel) * factor
            tolerance = joint.get("static_tolerance", PROJECT["safety"]["angle"]["static_tolerance"])
            in_range = lo - tolerance <= pos <= hi + tolerance
            ok = ok and in_range
            flag = "OK" if in_range else f"⚠ 超出 [{lo:+.2f},{hi:+.2f}]"
            print(f"{name:<9}{mid:>6}{fid:>8}{pos:>12.4f}{velocity:>10.3f}"
                  f"{float(state.torq):>9.3f}{float(state.t_mos):>9.1f}  {flag}")

        # ---- 电机固件里的控制参数（寄存器 21/22/23；只读，不写） ----
        # 为什么必须知道 TMAX：MIT 模式下力矩命令由电机按 TMAX 归一化并**在固件里**限幅。
        # 软件侧的 torque_limit 若大于 TMAX，多出来的部分电机直接削掉 —— 看起来"没生效"。
        print(f"\n{'关节':<9}{'PMAX(rad)':>11}{'VMAX(rad/s)':>13}{'TMAX(N·m)':>11}{'CTRL_MODE':>11}")
        for name in JOINTS:
            motor = motors[name]
            def _rd(fn, reg):
                for _ in range(SETTINGS["register_attempts"]):
                    try:
                        return fn(reg, SETTINGS["register_timeout_ms"])
                    except Exception:  # noqa: BLE001 - 个别电机不支持就打印 ?
                        time.sleep(SETTINGS["register_retry_sec"])
                return None
            pmax = _rd(motor.get_register_f32, 21)
            vmax = _rd(motor.get_register_f32, 22)
            tmax = _rd(motor.get_register_f32, 23)
            mode = _rd(motor.get_register_u32, 10)
            f = lambda v, nd: ("?" if v is None else f"{v:.{nd}f}")  # noqa: E731
            print(f"{name:<9}{f(pmax,3):>11}{f(vmax,3):>13}{f(tmax,3):>11}"
                  f"{('?' if mode is None else int(mode)):>11}")
        print("  说明：TMAX 是电机固件里力矩映射/限幅的上限；MIT 模式下软件 torque_limit")
        print("        必须 ≤ TMAX，否则超出部分会被电机静默削掉（软件限幅形同虚设）。")

        if not ok:
            print("\n⚠ 有电机没反馈或角度超出 URDF 限位。")
            print("  若只是角度超限：可能是零位没标定，或关节方向定义与真机相反 ——")
            print("  先别使能，用 b601_hardware 的 direction / zero_offset 参数对齐，不要改 URDF。")

        if args.enable:
            print("\n==> 使能测试（会立刻失能）…")
            try:
                ctrl.enable_all()
                time.sleep(SETTINGS["enable_hold_sec"])
                for name, motor in motors.items():
                    state = motor.get_state()
                    if state is not None:
                        print(f"    使能后 {name:<9} pos={float(state.pos):+.4f}")
                ctrl.disable_all()
                print("✅ 使能/失能流程正常（电机现在已失能，可以手推）")
            except Exception as exc:  # noqa: BLE001
                print(f"❌ 使能失败：{exc}")
                try:
                    ctrl.disable_all()
                except Exception:  # noqa: BLE001
                    pass
                return 1

        print("\n✅ 探针结束。下一步：把 ros2_control.xacro 里 enable_motors 保持 false，"
              "跑 ros2 launch b601_bringup bringup.launch.py hardware:=dm 看只读模式。")
        return 0
    finally:
        try:
            ctrl.close()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    sys.exit(main())
