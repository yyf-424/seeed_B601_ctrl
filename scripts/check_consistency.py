#!/usr/bin/env python3
"""check_consistency.py —— 跨文件的静态一致性检查（不需要装 ros2_control / MoveIt）。

为什么需要它：
    URDF、controller yaml、SRDF、moveit_controllers 这四个文件里的关节名必须完全对得上。
    对不上时 ROS 不会在 build 阶段报错，而是等到运行时报
    "joint not found" / "resource is already claimed" / "no IK for group"，
    一晚上就没了。所以这里在 build 之前先把它们交叉比对一遍。

用法：
    python3 scripts/check_consistency.py <expanded.urdf> \
        --project-config ros2_ws/src/b601_description/config/b601_project.yaml \
        --controllers ros2_ws/src/b601_bringup/config/b601_controllers.yaml \
        --moveit-controllers ros2_ws/src/b601_moveit_config/config/moveit_controllers.yaml \
        --srdf ros2_ws/src/b601_moveit_config/config/b601_dm.srdf

除了关节名交叉比对，还会检查「唯一真值 → 派生文件」是否同步：
    真值是 b601_project.yaml；派生文件由 scripts/sync_config.py 生成，
    这里调用它的 --check 模式，任何一处忘了同步都会报错。

退出码：0 = 全部一致；1 = 有错误（errors 非空）。warnings 不影响退出码。
"""

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


def load_urdf(path):
    root = ET.parse(path).getroot()
    links = {el.get("name") for el in root.findall("link")}
    joints = {}
    for el in root.findall("joint"):
        name = el.get("name")
        parent = el.find("parent")
        child = el.find("child")
        limit = el.find("limit")
        mimic = el.find("mimic")
        joints[name] = {
            "type": el.get("type"),
            "parent": parent.get("link") if parent is not None else None,
            "child": child.get("link") if child is not None else None,
            "lower": float(limit.get("lower")) if limit is not None and limit.get("lower") else None,
            "upper": float(limit.get("upper")) if limit is not None and limit.get("upper") else None,
            "velocity": float(limit.get("velocity")) if limit is not None and limit.get("velocity") else None,
            "effort": float(limit.get("effort")) if limit is not None and limit.get("effort") else None,
            "mimic": mimic.get("joint") if mimic is not None else None,
        }
    inertials = {}
    for el in root.findall("link"):
        inertial = el.find("inertial")
        if inertial is not None:
            mass = inertial.find("mass")
            inertials[el.get("name")] = float(mass.get("value")) if mass is not None else None
    ros2_control = {}
    for system in root.findall("ros2_control"):
        plugin = system.find("hardware/plugin")
        entry = {"plugin": plugin.text.strip() if plugin is not None else None, "joints": {}}
        for joint in system.findall("joint"):
            entry["joints"][joint.get("name")] = {
                "command": [ci.get("name") for ci in joint.findall("command_interface")],
                "state": [si.get("name") for si in joint.findall("state_interface")],
            }
        ros2_control[system.get("name")] = entry
    return {"links": links, "joints": joints, "masses": inertials, "ros2_control": ros2_control}


def check_ros2_control(urdf, errors, warnings):
    if not urdf["ros2_control"]:
        errors.append("URDF 里没有 <ros2_control> 块：xacro 的 hardware 分支可能没展开")
        return
    for name, system in urdf["ros2_control"].items():
        if not system["plugin"]:
            errors.append(f"<ros2_control name={name}> 没有 <plugin>")
        for joint_name, ifaces in system["joints"].items():
            info = urdf["joints"].get(joint_name)
            if info is None:
                errors.append(f"ros2_control 引用了不存在的关节 {joint_name}")
                continue
            if info["type"] == "fixed":
                errors.append(f"ros2_control 里出现了 fixed 关节 {joint_name}")
            if not ifaces["command"] and not ifaces["state"]:
                errors.append(f"关节 {joint_name} 既没有命令也没有状态接口")
            if info["mimic"] and ifaces["command"]:
                errors.append(f"mimic 关节 {joint_name} 不应该有命令接口（它跟随 {info['mimic']}）")
        # 反向检查：URDF 里能动的关节都该出现在 ros2_control 里
        for joint_name, info in urdf["joints"].items():
            if info["type"] in ("revolute", "prismatic", "continuous") and joint_name not in system["joints"]:
                warnings.append(f"可动关节 {joint_name} 没有出现在 ros2_control 里（不打算控制就忽略）")


def check_controllers(urdf, path, errors, warnings):
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cm = cfg.get("controller_manager", {}).get("ros__parameters", {})
    if not cm:
        errors.append(f"{path} 里没有 controller_manager.ros__parameters")
        return

    declared = {k for k, v in cm.items() if isinstance(v, dict) and "type" in v}
    controlled_joints = set()
    for system in urdf["ros2_control"].values():
        controlled_joints |= set(system["joints"])

    claim = {}
    for name in declared:
        params = cfg.get(name, {}).get("ros__parameters", {})
        joints = params.get("joints", [])
        for joint in joints:
            if joint not in urdf["joints"]:
                errors.append(f"controller {name} 引用了不存在的关节 {joint}")
            elif joint not in controlled_joints:
                errors.append(f"controller {name} 的关节 {joint} 在 ros2_control 里没有接口")
            if joint in claim:
                errors.append(f"关节 {joint} 同时被 {claim[joint]} 和 {name} 占用（会争抢 command interface）")
            claim[joint] = name

    if "joint_state_broadcaster" not in declared:
        errors.append("缺少 joint_state_broadcaster：/joint_states 不会发布，MoveIt 与 RViz 都会瞎")


def check_mit_controllers(urdf, path, errors):
    """MIT 力控实例表：每个可控关节必须且只能属于一个实例（裸关节 = 力矩恒 0）。

    为什么要单独查一遍：sync_config 的校验针对**真值**，这里查**生成物**。
    两道关一起才挡得住"手改了生成文件"或"真值里 profile 写漏一个关节"。
    """
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cm = cfg.get("controller_manager", {}).get("ros__parameters", {})
    if not cm:
        errors.append(f"{path} 里没有 controller_manager.ros__parameters")
        return
    instances = {k: v for k, v in cm.items() if isinstance(v, dict) and "type" in v
                 and k != "joint_state_broadcaster"}
    if not instances:
        errors.append(f"{path} 里没有 MIT 控制器实例（controller_manager 只声明了 broadcaster）")
        return

    required_commands = {"position", "velocity", "kp", "kd", "effort"}
    effort_joints = set()
    for system in urdf["ros2_control"].values():
        for name, joint in system["joints"].items():
            commands = set(joint.get("command", []))
            if "effort" in commands:
                effort_joints.add(name)
                if commands != required_commands:
                    errors.append(f"{name} MIT 接口不完整：{sorted(commands)}")
    if not effort_joints:
        # 位置模式的 URDF 展开：MIT 实例表不适用，跳过而不是报错
        return

    claim = {}
    for name, decl in instances.items():
        params = cfg.get(name, {}).get("ros__parameters", {})
        joints = params.get("joints", [])
        chain = params.get("chain_joints", [])
        if not joints:
            errors.append(f"MIT 实例 {name} 没有 joints")
        for joint in joints:
            if joint not in urdf["joints"]:
                errors.append(f"MIT 实例 {name} 引用了不存在的关节 {joint}")
            if joint in claim:
                errors.append(f"关节 {joint} 同时被 {claim[joint]} 与 {name} 声明（会互相抢命令接口）")
            claim[joint] = name
        # 逐关节数组长度必须与 joints 一致（长度错 = 参数静默失效）
        for key, val in params.items():
            if isinstance(val, list) and key not in ("joints", "chain_joints", "state_interfaces"):
                if key == "gravity_params":
                    continue
                if len(val) != len(joints):
                    errors.append(
                        f"MIT 实例 {name} 的 {key} 长度 {len(val)} ≠ joints 个数 {len(joints)}")
        for cname in chain:
            if cname not in urdf["joints"]:
                errors.append(f"MIT 实例 {name} 的 chain_joints 引用了不存在的关节 {cname}")

    for joint in sorted(effort_joints):
        if joint not in claim:
            errors.append(
                f"关节 {joint} 有 effort 命令接口却没有任何 MIT 实例声明它 —— "
                f"力矩恒 0（裸关节），使能后会直接塌或不受控")

    # 声明顺序检查：数组参数必须与 joints 顺序对齐（防止"参数按 6 元写、实例只控 1 个"）
    for decl in instances:
        params = cfg.get(decl, {}).get("ros__parameters", {})
        if not params.get("joints"):
            continue
        for key in ("kp_hold", "kd_hold", "kp_move", "kd_move", "friction", "torque_limit"):
            val = params.get(key)
            if isinstance(val, list) and len(val) != len(params["joints"]):
                errors.append(f"MIT 实例 {decl} 的 {key} 与 joints 长度不一致")


def check_moveit_controllers(urdf, path, ros_controller_names, errors):
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    manager = cfg.get("moveit_controller_manager")
    if not manager:
        errors.append(f"{path} 缺少 moveit_controller_manager")
    simple = cfg.get("moveit_simple_controller_manager", {})
    names = simple.get("controller_names", [])
    if not names:
        errors.append(f"{path} 里 controller_names 为空")
    for name in names:
        if name not in ros_controller_names:
            errors.append(f"MoveIt 想用 controller {name}，但 controller_manager 里没有这个名字")
        entry = simple.get(name, {})
        for joint in entry.get("joints", []):
            if joint not in urdf["joints"]:
                errors.append(f"moveit controller {name} 引用了不存在的关节 {joint}")


def check_srdf(urdf, path, errors):
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        errors.append(f"{path} 根元素不是 <robot>")
    groups = {g.get("name"): g for g in root.findall("group")}
    if not groups:
        errors.append(f"{path} 没有规划组")

    for name, group in groups.items():
        for chain in group.findall("chain"):
            base, tip = chain.get("base_link"), chain.get("tip_link")
            for link in (base, tip):
                if link not in urdf["links"]:
                    errors.append(f"SRDF 组 {name} 的 chain 引用了不存在的 link {link}")
        for joint in group.findall("joint"):
            jname = joint.get("name")
            if jname not in urdf["joints"]:
                errors.append(f"SRDF 组 {name} 引用了不存在的关节 {jname}")
            elif urdf["joints"][jname]["type"] == "fixed":
                errors.append(f"SRDF 组 {name} 包含 fixed 关节 {jname}")

    for state in root.findall("group_state"):
        group_name = state.get("group")
        if group_name not in groups:
            errors.append(f"group_state {state.get('name')} 指向不存在的组 {group_name}")
        for joint in state.findall("joint"):
            jname = joint.get("name")
            info = urdf["joints"].get(jname)
            if info is None:
                errors.append(f"group_state {state.get('name')} 引用了不存在的关节 {jname}")
                continue
            value = float(joint.get("value"))
            if info["lower"] is not None and not (info["lower"] - 1e-6 <= value <= info["upper"] + 1e-6):
                errors.append(
                    f"group_state {state.get('name')} 的 {jname}={value} 超出 URDF 限位 "
                    f"[{info['lower']}, {info['upper']}]"
                )

    for dc in root.findall("disable_collisions"):
        for attr in ("link1", "link2"):
            if dc.get(attr) not in urdf["links"]:
                errors.append(f"disable_collisions 引用了不存在的 link {dc.get(attr)}")

    ee = root.findall("end_effector")
    for item in ee:
        parent_link = item.get("parent_link")
        if parent_link not in urdf["links"]:
            errors.append(f"end_effector 的 parent_link {parent_link} 不存在")
        if item.get("group") not in groups:
            errors.append(f"end_effector 的 group {item.get('group')} 不存在")

    if "world" in {item.get("parent_frame") for item in root.findall("virtual_joint")}:
        pass  # 本项目把 world 放在 URDF 里，不需要 virtual_joint，这里不加检查


def check_urdf_matches_config(urdf, config_path, errors):
    """展开后的 URDF 限位必须与唯一真值一致（防止 xacro 没读到配置、或读了旧副本）。"""
    with open(config_path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    joints = cfg.get("joints", {})
    for name, expected in joints.items():
        info = urdf["joints"].get(name)
        if info is None:
            errors.append(f"真值里有 {name}，但 URDF 里没有这个关节")
            continue
        for key, urdf_key in (("min_pos", "lower"), ("max_pos", "upper"),
                              ("max_velocity", "velocity"), ("max_effort", "effort")):
            if key not in expected:
                continue
            have = info[urdf_key]
            want = float(expected[key])
            if have is None or abs(have - want) > 1e-6:
                errors.append(
                    f"{name} 的 URDF {urdf_key}={have} 与真值 {key}={want} 不一致"
                    "（xacro 是否漏传 project_config_file？）"
                )
    # 手臂关节顺序
    arm = cfg.get("robot", {}).get("arm_joints", [])
    for name in arm:
        if name not in urdf["ros2_control"].get(
                cfg.get("robot", {}).get("ros2_control_name", "b601"), {}).get("joints", {}):
            errors.append(f"真值的 arm_joints 里的 {name} 没有出现在 ros2_control 接口里")


def check_derived_sync(repo_root, errors):
    """派生配置文件是否与真值同步（调用 sync_config.py --check）。"""
    import subprocess

    script = repo_root / "scripts" / "sync_config.py"
    if not script.is_file():
        errors.append(f"找不到 {script}，无法校验派生配置")
        return
    result = subprocess.run(
        [sys.executable, str(script), "--check"], capture_output=True, text=True, check=False
    )
    if result.returncode != 0:
        detail = (result.stdout or result.stderr).strip().splitlines()
        for line in detail:
            errors.append("派生配置不同步：" + line.strip(" ·"))


def check_limits(urdf, errors):
    for name, info in urdf["joints"].items():
        if info["type"] not in ("revolute", "prismatic", "continuous"):
            continue
        if info["type"] != "continuous":
            if info["lower"] is None or info["upper"] is None:
                errors.append(f"{name} 缺 limit.lower/upper")
            elif info["lower"] >= info["upper"]:
                errors.append(f"{name} 的限位区间非法：[{info['lower']}, {info['upper']}]")
        if not info["velocity"] or info["velocity"] <= 0:
            errors.append(f"{name} 缺 velocity 限位（MoveIt 时间参数化会失败）")
        if not info["effort"] or info["effort"] <= 0:
            errors.append(f"{name} 缺 effort 限位")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf")
    parser.add_argument("--project-config", help="唯一真值 b601_project.yaml")
    parser.add_argument("--repo-root", default=".", help="仓库根目录（用于校验派生配置）")
    parser.add_argument("--controllers")
    parser.add_argument("--moveit-controllers")
    parser.add_argument("--srdf")
    parser.add_argument("--mit-controllers", help="MIT 力控控制器 yaml（生成物）")
    args = parser.parse_args()

    urdf = load_urdf(args.urdf)
    errors, warnings = [], []

    print("== URDF 概览 ==")
    movable = [j for j, i in urdf["joints"].items() if i["type"] in ("revolute", "prismatic", "continuous")]
    total_mass = sum(m for m in urdf["masses"].values() if m)
    print(f"  link {len(urdf['links'])} 个 / joint {len(urdf['joints'])} 个 / 可动关节 {len(movable)} 个")
    print(f"  总质量 {total_mass:.3f} kg（含夹爪，重力补偿辨识的起点）")
    print(f"  可动关节: {', '.join(movable)}")

    check_limits(urdf, errors)
    check_ros2_control(urdf, errors, warnings)
    if args.project_config:
        check_urdf_matches_config(urdf, args.project_config, errors)
        check_derived_sync(Path(args.repo_root), errors)

    ros_controller_names = set()
    if args.controllers:
        check_controllers(urdf, args.controllers, errors, warnings)
        with open(args.controllers, encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        cm = cfg.get("controller_manager", {}).get("ros__parameters", {})
        ros_controller_names = {k for k, v in cm.items() if isinstance(v, dict) and "type" in v}
    if args.moveit_controllers:
        check_moveit_controllers(urdf, args.moveit_controllers, ros_controller_names, errors)
    if args.srdf:
        check_srdf(urdf, args.srdf, errors)
    if args.mit_controllers:
        check_mit_controllers(urdf, args.mit_controllers, errors)

    for w in warnings:
        print(f"  [warn] {w}")
    for e in errors:
        print(f"  [FAIL] {e}")

    if errors:
        print(f"\n❌ 发现 {len(errors)} 个一致性问题")
        return 1
    print(f"\n✅ 一致性检查通过（{len(warnings)} 条警告）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
