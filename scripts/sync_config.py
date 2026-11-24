#!/usr/bin/env python3
"""sync_config.py —— 把「唯一真值」同步到全项目所有派生配置文件。

    ./scripts/sync_config.py            # 生成 / 更新全部派生文件
    ./scripts/sync_config.py --check    # 只检查是否已同步（verify.sh 用；不一致退出码 1）
    ./scripts/sync_config.py --diff     # 打印差异（不写盘）

真值：ros2_ws/src/b601_description/config/b601_project.yaml

派生（带 DO NOT EDIT 头，手改会被下一次 sync 覆盖）：
    b601_description/config/initial_positions.yaml     上电/启动姿态
    b601_bringup/config/b601_controllers.yaml          位置模式控制器
    b601_moveit_config/config/joint_limits.yaml        MoveIt 速度/加速度
    b601_safety/config/safety_params.yaml              安全监控节点参数

不生成（仍手写）：
    b601_bringup/config/b601_mit_controllers.yaml      MIT 力控/重力补偿控制器
      —— 这条线（b601_mit_controller + b601_gravity_id）真机复测通过后再迁入真值，
         见本文件顶部说明与 docs/test-report.md「遗留」。

不派生的（运行时直接读真值，因此永远同步）：
    b601_description/urdf/inc/b601_links.xacro         关节 limit
    b601_description/urdf/inc/ros2_control.xacro       命令/状态接口 + 电机参数
    b601_demos/b601_demos/trajectory_utils.py          越界拒发用的限位表

为什么用「生成」而不是让每个包都去读同一个 yaml：
    controller_manager / MoveIt 只认标准的 yaml 参数文件格式，必须落盘；
    生成 + 校验（而不是手写）才能保证「改一处、全同步、且可验证」。
"""

from __future__ import annotations

import argparse
import difflib
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
WS_SRC = REPO_ROOT / "ros2_ws" / "src"
TRUTH = WS_SRC / "b601_description" / "config" / "b601_project.yaml"

GENERATED_NOTE = (
    "# ============================================================================\n"
    "# ⚠ 本文件由 scripts/sync_config.py 自动生成，请勿手工编辑。\n"
    "#    要改参数请改：ros2_ws/src/b601_description/config/b601_project.yaml\n"
    "#    改完执行：  ./scripts/sync_config.py\n"
    "# ============================================================================\n"
)


# --------------------------------------------------------------------------- 校验
def validate(cfg: dict) -> list[str]:
    """配置自身的一致性检查。返回错误列表（空 = 通过）。"""
    errors: list[str] = []
    joints = cfg.get("joints", {})
    robot = cfg.get("robot", {})
    safety = cfg.get("safety", {})

    if not joints:
        errors.append("joints 段为空")
        return errors

    arm = robot.get("arm_joints", [])
    if len(arm) != 6:
        errors.append(f"robot.arm_joints 应为 6 个，实际 {len(arm)}")
    for name in arm + [robot.get("gripper_joint", ""), robot.get("mimic_joint", "")]:
        if name and name not in joints:
            errors.append(f"robot 段引用了 joints 里不存在的关节 {name}")

    for name, j in joints.items():
        for key in ("min_pos", "max_pos", "max_velocity", "max_effort", "startup"):
            if key not in j:
                errors.append(f"关节 {name} 缺 {key}")
        if j.get("min_pos", 0) >= j.get("max_pos", 0):
            errors.append(f"关节 {name} 限位区间非法：[{j.get('min_pos')}, {j.get('max_pos')}]")
        if j.get("max_velocity", 0) <= 0:
            errors.append(f"关节 {name} 的 max_velocity 必须 > 0")
        start = j.get("startup")
        if start is not None and not (j["min_pos"] <= start <= j["max_pos"]):
            errors.append(f"关节 {name} 的启动姿态 {start} 不在限位内")
        jump = j.get("feedback_jump_limit")
        if jump is None:
            errors.append(f"关节 {name} 缺 feedback_jump_limit（防放宽角速度时守卫也跟着放松）")
        elif float(jump) <= 0:
            errors.append(f"关节 {name} 的 feedback_jump_limit 必须 > 0（当前 {jump}）")

        if j.get("control"):
            motor = j.get("motor")
            if not motor:
                errors.append(f"关节 {name} 有命令接口但没有 motor 段")
            else:
                if motor.get("scale", 0) == 0:
                    errors.append(f"关节 {name} 的 motor.scale 不能为 0")
                if motor.get("direction") not in (1.0, -1):
                    errors.append(f"关节 {name} 的 motor.direction 必须是 ±1")

    # 安全阈值自洽
    vel = safety.get("velocity", {})
    if vel.get("warn_ratio", 0) >= vel.get("estop_ratio", 0):
        errors.append("safety.velocity.warn_ratio 必须小于 estop_ratio（否则警告永远看不见）")
    if safety.get("angle", {}).get("warn_margin", 0) <= 0:
        errors.append("safety.angle.warn_margin 必须 > 0")
    if float(safety.get("angle", {}).get("static_tolerance", 0.05)) < 0:
        errors.append("safety.angle.static_tolerance 必须 >= 0")
    estop = safety.get("estop", {})
    hw_name = estop.get("hardware_component")
    if hw_name != robot.get("ros2_control_name"):
        errors.append(
            f"safety.estop.hardware_component={hw_name} 与 robot.ros2_control_name="
            f"{robot.get('ros2_control_name')} 不一致，急停会找不到硬件"
        )
    for cname in estop.get("deactivate_controllers", []) or []:
        if cname == "*":
            continue  # 通配：停掉当时所有 active 控制器
        known = {c.get("name") for c in cfg.get("controllers", {}).values() if isinstance(c, dict)}
        if cname not in known:
            errors.append(f"safety.estop.deactivate_controllers 里的 {cname} 不是已定义的控制器")
    return errors


# --------------------------------------------------------------------- 渲染各文件
def dump_yaml(data: dict) -> str:
    return yaml.safe_dump(data, sort_keys=False, allow_unicode=True, default_flow_style=False)


def render_initial_positions(cfg: dict) -> str:
    positions = {name: float(j["startup"]) for name, j in cfg["joints"].items()}
    head = (
        GENERATED_NOTE
        + "# 上电/启动时的关节角（rad / m）。被 ros2_control.xacro 的 state_interface\n"
        "# initial_value 使用，决定\n"
        "#   - mock 后端启动后 /joint_states 的初值\n"
        "#   - 真机后端上电时认为关节在哪儿（必须与实测一致，否则第一次写命令会跳）\n"
        "#   - 安全节点判断「是否已离开启动位形」的基线\n"
    )
    return head + dump_yaml({"initial_positions": positions})


def _controller_block(name: str, ctrl: dict) -> tuple[dict, dict]:
    """返回 (controller_manager 里的声明, 控制器自身的 ros__parameters)。"""
    declaration = {"type": ctrl["type"]}
    params: dict = {"joints": list(ctrl.get("joints", []))}
    if ctrl.get("command_interfaces"):
        params["command_interfaces"] = list(ctrl["command_interfaces"])
    if ctrl.get("state_interfaces"):
        params["state_interfaces"] = list(ctrl["state_interfaces"])
    params.update(ctrl.get("params", {}))
    return declaration, params


def render_controllers(cfg: dict) -> str:
    ctrls = cfg["controllers"]
    cm_params: dict = {"update_rate": ctrls["update_rate"]}
    body: dict = {}

    broadcaster = ctrls["joint_state_broadcaster"]
    cm_params["joint_state_broadcaster"] = {"type": broadcaster["type"]}
    body["joint_state_broadcaster"] = {"ros__parameters": dict(broadcaster.get("params", {}))}

    for key in ("arm", "gripper"):
        ctrl = ctrls[key]
        declaration, params = _controller_block(ctrl["name"], ctrl)
        cm_params[ctrl["name"]] = declaration
        body[ctrl["name"]] = {"ros__parameters": params}

    head = (
        GENERATED_NOTE
        + "# 位置模式控制器（hardware:=mock|dm，command_mode=pos_vel 时用这套）\n"
        + "# 力控模式用 b601_mit_controllers.yaml\n"
    )
    data = {"controller_manager": {"ros__parameters": cm_params}}
    data.update(body)
    return head + dump_yaml(data)


def render_joint_limits(cfg: dict) -> str:
    moveit = cfg["moveit"]
    limits: dict = {}
    for name, j in cfg["joints"].items():
        if not j.get("control"):
            continue  # mimic 关节不参与规划
        acc = float(j["max_acceleration"])
        limits[name] = {
            "has_velocity_limits": True,
            "max_velocity": float(j["max_velocity"]),
            "has_acceleration_limits": True,
            "max_acceleration": acc,
            "has_deceleration_limits": True,
            "max_deceleration": -acc,
        }
    data = {
        "default_velocity_scaling_factor": moveit["default_velocity_scaling_factor"],
        "default_acceleration_scaling_factor": moveit["default_acceleration_scaling_factor"],
        "joint_limits": limits,
    }
    head = (
        GENERATED_NOTE
        + "# MoveIt 规划限速：时间参数化的依据。速度上限 = URDF limit.velocity（同一个来源）。\n"
    )
    return head + dump_yaml(data)


def render_safety_params(cfg: dict) -> str:
    safety = cfg["safety"]
    joints = cfg["joints"]
    names = list(joints.keys())

    default_margin = float(safety["angle"]["warn_margin"])
    params: dict = {
        "joint_names": names,
        "pos_min": [float(joints[n]["min_pos"]) for n in names],
        "pos_max": [float(joints[n]["max_pos"]) for n in names],
        "vel_max": [float(joints[n]["max_velocity"]) for n in names],
        "startup_pose": [float(joints[n]["startup"]) for n in names],
        # 逐关节的软警告配置（夹爪关掉"接近限位/离开启动位形"，否则开合会一直挂警告）
        "warn_pos_margin": [float(joints[n].get("warn_pos_margin", default_margin))
                            for n in names],
        "warn_near_limit": [bool(joints[n].get("warn_near_limit", True)) for n in names],
        "warn_pose_deviation_joint": [bool(joints[n].get("warn_pose_deviation", True))
                                      for n in names],
        "evaluate_rate_hz": safety["evaluate_rate_hz"],
        "publish_rate_hz": safety["publish_rate_hz"],
        "feedback_timeout_sec": safety["feedback_timeout_sec"],
        "estop_on_feedback_timeout": safety["estop_on_feedback_timeout"],
        "startup_grace_sec": safety["startup_grace_sec"],
        "estop_pos_margin": safety["angle"]["estop_margin"],
        # 静止零位漂移容差（同 hardware 激活检查用的那个值）
        # 静止零位漂移容差：逐关节列表（夹爪单位 m，真值里单独覆盖成 0.002）
        "static_tolerance": [float(joints[n].get(
            "static_tolerance", safety["angle"].get("static_tolerance", 0.05))) for n in names],
        "warn_vel_ratio": safety["velocity"]["warn_ratio"],
        "estop_vel_ratio": safety["velocity"]["estop_ratio"],
        "estop_vel_absolute": safety["velocity"]["estop_absolute"],
        "warn_on_pose_deviation": safety["pose"]["warn_on_deviation"],
        "warn_pose_threshold": safety["pose"]["warn_deviation"],
        "status_topic": safety["status_topic"],
        "clear_service": safety["clear_service"],
        "deactivate_controllers": list(safety["estop"].get("deactivate_controllers") or []),
        "deactivate_hardware": safety["estop"]["deactivate_hardware"],
        "hardware_component": safety["estop"]["hardware_component"],
        "service_timeout_sec": safety["estop"]["service_timeout_sec"],
        "latch": safety["estop"]["latch"],
    }
    data = {safety.get("node_name", "b601_safety_monitor"): {"ros__parameters": params}}
    head = (
        GENERATED_NOTE
        + "# 安全监控节点参数。限位/速度上限来自 joints 段，阈值来自 safety 段。\n"
        + "# 想永久调整阈值请改 b601_project.yaml，不要手改本文件。\n"
    )
    return head + dump_yaml(data)


def render_all(cfg: dict) -> dict[Path, str]:
    return {
        WS_SRC / "b601_description" / "config" / "initial_positions.yaml":
            render_initial_positions(cfg),
        WS_SRC / "b601_bringup" / "config" / "b601_controllers.yaml":
            render_controllers(cfg),
        WS_SRC / "b601_moveit_config" / "config" / "joint_limits.yaml":
            render_joint_limits(cfg),
        WS_SRC / "b601_safety" / "config" / "safety_params.yaml":
            render_safety_params(cfg),
    }


# ------------------------------------------------------------------------- 主流程
def main() -> int:
    parser = argparse.ArgumentParser(description="把 b601_project.yaml 同步到全部派生配置")
    parser.add_argument("--check", action="store_true", help="只检查是否已同步（不写盘）")
    parser.add_argument("--diff", action="store_true", help="打印差异（不写盘）")
    parser.add_argument("--quiet", action="store_true", help="成功时不打印文件清单")
    args = parser.parse_args()

    if not TRUTH.is_file():
        print(f"❌ 找不到真值配置：{TRUTH}")
        return 1
    with TRUTH.open(encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    errors = validate(cfg)
    if errors:
        print(f"❌ 配置自身有 {len(errors)} 个问题：")
        for e in errors:
            print(f"   · {e}")
        return 1

    rendered = render_all(cfg)
    drifted: list[Path] = []

    for path, content in rendered.items():
        old = path.read_text(encoding="utf-8") if path.is_file() else ""
        if old == content:
            continue
        drifted.append(path)
        if args.check or args.diff:
            if args.diff:
                diff = difflib.unified_diff(
                    old.splitlines(keepends=True), content.splitlines(keepends=True),
                    fromfile=f"{path}（磁盘）", tofile=f"{path}（应按真值生成）",
                )
                sys.stdout.writelines(diff)
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")

    rel = [str(p.relative_to(REPO_ROOT)) for p in drifted]

    if args.check:
        if drifted:
            print(f"❌ 有 {len(drifted)} 个派生文件与真值不同步：")
            for r in rel:
                print(f"   · {r}")
            print("   → 跑 ./scripts/sync_config.py 同步")
            return 1
        print("✅ 所有派生配置与 b601_project.yaml 同步")
        return 0

    if args.diff:
        return 1 if drifted else 0

    if drifted:
        print(f"✅ 已生成 {len(drifted)} 个派生文件：")
        for r in rel:
            print(f"   · {r}")
    else:
        print("✅ 派生配置本来就是最新的（无需改动）")
    if not args.quiet:
        print(f"\n真值：{TRUTH.relative_to(REPO_ROOT)}")
        print("下一步：./scripts/verify.sh   （一致性检查会验证派生文件与真值匹配）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
