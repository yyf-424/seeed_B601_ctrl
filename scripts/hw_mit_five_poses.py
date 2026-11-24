#!/usr/bin/env python3
"""Run the recorded six-axis and gripper MIT acceptance sequence."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListHardwareComponents, SetHardwareComponentState, SwitchController
from lifecycle_msgs.msg import State
from moveit_msgs.srv import GetStateValidity
import rclpy

from mit_five_poses import (
    CONFIG, CRITERIA, NAMES, OBSERVED, PLAN, FivePoseHarness,
    check_plan, checked_sample, score_hardware_pose, validate_configuration,
)

from project_config import ROOT

HARDWARE = PLAN["hardware_retest"]
OUT = ROOT / HARDWARE["output_dir"]
HOLD_FILE = OUT / "HOLD_HARDWARE"
COLLISION_LAUNCH = ROOT / "scripts/mit_collision_meshes.launch.py"

CONTROLLERS = ["joint_state_broadcaster", "b601_mit_arm", "b601_mit_gripper"]
ARM = CONFIG["robot"]["arm_joints"]
GRIPPER = CONFIG["robot"]["gripper_joint"]
GRIPPER_INDEX = len(ARM)
GRIPPER_MOTOR = CONFIG["joints"][GRIPPER]["motor"]
MIT = CONFIG["controllers"]["mit"]
GRIPPER_PROFILE = MIT["instances"]["b601_mit_gripper"]["profile"]
GRIPPER_GAINS = MIT["profiles"][GRIPPER_PROFILE]["per_joint"][GRIPPER]
PARK_TOLERANCE = CONFIG["recenter"]["position_tolerance_rad"]
PARK = [CONFIG["recenter"]["joints"][name] for name in ARM] + [
    HARDWARE["gripper_parking_target_m"],
]
PERIOD = 1 / PLAN["publish_rate_hz"]


def bringup_config(name):
    return Path(get_package_share_directory("b601_bringup")) / "config" / name


def launch(log, *arguments):
    with (OUT / log).open("w") as output:
        return subprocess.Popen(["ros2", "launch", *arguments], stdout=output,
                                stderr=subprocess.STDOUT, start_new_session=True)


def bringup(enable):
    check_cancelled()
    device = CONFIG["hardware"]["dm"]["device"]
    if subprocess.run(["fuser", device], capture_output=True).returncode != 1:
        raise RuntimeError(f"未确认串口空闲：{device}")
    mode = "hardware" if enable else "readonly"
    return launch(f"{mode}_bringup.log", "b601_bringup", "bringup.launch.py",
                  "hardware:=dm", f"dm_enable_motors:={str(enable).lower()}",
                  "dm_command_mode:=mit_impedance", "use_rviz:=false", "use_safety:=true",
                  f"controllers_file:={bringup_config('b601_mit_controllers.yaml')}",
                  f"safety_params_file:={bringup_config('b601_mit_safety_verified.yaml')}",
                  f'dm_log_csv:={OUT / (mode + "_dm.csv")}')


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=PLAN["shutdown_timeout_sec"])


def ready(node, process, label):
    deadline = time.monotonic() + PLAN["controller_timeout_sec"]
    while time.monotonic() < deadline:
        log = (OUT / f"{label}_bringup.log").read_text(errors="replace")
        log = re.sub(r"\x1b\[[0-9;]*m", "", log)
        if process.poll() is not None or "process has died" in log:
            raise RuntimeError(f"驱动启动异常，见 {label}_bringup.log")
        if all(f"Configured and activated {name}" in log for name in CONTROLLERS):
            break
        node.spin_for(PERIOD)
    else:
        raise RuntimeError("6+1 控制器未激活")
    if not node.wait_ready(PLAN["joint_state_timeout_sec"]):
        raise RuntimeError("没有完整的关节反馈")
    deadline = time.monotonic() + PLAN["controller_timeout_sec"]
    while node.status_level < 0 or not node.ff or not node.gripper_ff:
        if time.monotonic() >= deadline:
            raise RuntimeError("安全状态或前馈观测未就绪")
        node.spin_for(PERIOD)
    node.check()
    node.hold(HARDWARE["observation_pause_sec"])


def state(node, duration=HARDWARE["state_average_sec"]):
    latest = node.samples[-1][0]
    rows = [row for row in node.samples if latest - row[0] <= duration]
    return node.average(rows), rows


def in_range(q):
    return [
        min(max(value, CONFIG["joints"][name]["min_pos"]), CONFIG["joints"][name]["max_pos"])
        for name, value in zip(NAMES, q)
    ]


def precheck(node, client, label, a, b):
    samples = HARDWARE["collision_samples_per_transit"]
    for i in range(samples):
        q = [x + (y - x) * i / (samples - 1) for x, y in zip(a, b)]
        checked_sample(node, client, q, f"{label} sample {i + 1}/{samples}")
    return samples


class TrialCancelled(RuntimeError):
    pass


def check_cancelled():
    if HOLD_FILE.exists():
        raise TrialCancelled("收到停止实机试验的请求")


class TrialOverspeed(RuntimeError):
    pass


class Motion:
    def __init__(self, node, writer):
        self.node = node
        self.writer = writer
        self.reference = None
        self.recovering = False

    def monitor(self, duration, motion=True):
        node = self.node
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if not self.recovering:
                check_cancelled()
            first = len(node.samples)
            rclpy.spin_once(node, timeout_sec=min(PERIOD, max(0.0, end - time.monotonic())))
            node.check()
            if motion and len(node.samples) != first:
                for _, positions, velocities in node.samples[first:]:
                    arm_speed = max(map(abs, velocities[:GRIPPER_INDEX]))
                    grip_speed = abs(velocities[GRIPPER_INDEX])
                    if arm_speed > HARDWARE["estop_arm_speed_rad_s"]:
                        raise TrialOverspeed(f"手臂速度中止线触发：{velocities}")
                    if grip_speed > HARDWARE["gripper_measured_speed_ceiling_m_s"]:
                        raise RuntimeError(f"夹爪速度中止线触发：{velocities}")
                    error = self.reference[GRIPPER_INDEX] - positions[GRIPPER_INDEX]
                    if abs(error) > HARDWARE["gripper_tracking_ceiling_m"]:
                        raise RuntimeError(f"夹爪参考误差触发测试中止线：{error:.4f} m")
                    torque = (GRIPPER_GAINS["kp_move"] * error
                              - GRIPPER_GAINS["kd_move"] * velocities[GRIPPER_INDEX])
                    torque *= GRIPPER_MOTOR["effort_scale"]
                    if abs(torque) > HARDWARE["gripper_torque_ceiling_nm"]:
                        raise RuntimeError(f"夹爪估算力矩触发测试中止线：{torque:.3f} N·m")

    def move(self, label, start, goal, arm_slew, settle=PLAN["settle_sec"]):
        if not self.recovering:
            check_cancelled()
        node = self.node
        before, _ = state(node)
        delta = [b - a for a, b in zip(start, goal)]
        steps = max(HARDWARE["minimum_motion_samples"], math.ceil(1.875 / PERIOD * max(
            max(map(abs, delta[:6])) / arm_slew,
            abs(delta[GRIPPER_INDEX]) / HARDWARE["gripper_slew_m_s"])))
        tick = time.monotonic()
        index = len(node.samples)
        try:
            for step in range(1, steps + 1):
                if not self.recovering:
                    check_cancelled()
                u = step / steps
                blend = u**3 * (10 + u * (-15 + 6 * u))
                terminal = step == steps
                self.reference = goal if terminal else [a + blend * d for a, d in zip(start, delta)]
                node.command_path(self.reference, goal, terminal=terminal)
                tick = max(tick + PERIOD, time.monotonic() + PERIOD / 2)
                self.monitor(tick - time.monotonic())
            self.monitor(settle)
        finally:
            for t, q, v in node.samples[index:]:
                self.writer.writerow([label, t, *q, *v])
        after, final_rows = state(node, PLAN["settle_window_sec"])
        rows = node.samples[index:]
        actual = [b - a for a, b in zip(before, after)]
        signed = [value * math.copysign(1, d) for value, d in zip(actual, delta)]
        end_speed = [sum(abs(row[2][i]) for row in final_rows) / len(final_rows)
                     for i in range(len(OBSERVED))]
        error = [abs(x - y) for x, y in zip(goal, after)]
        mimic_error = max(abs(q[6] + q[7]) for _, q, _ in rows)
        result = {
            "name": label, "target": goal, "before": before, "after": after, "actual": actual,
            "min_signed_arm_movement_rad": min(signed[:6]), "signed_gripper_movement_m": signed[6],
            "arm_max_error_rad": max(error[:6]), "gripper_error_m": error[6],
            "arm_peak_speed_rad_s": max(abs(x) for _, _, v in rows for x in v[:6]),
            "gripper_peak_speed_m_s": max(abs(v[6]) for _, _, v in rows),
            "arm_end_speed_rad_s": max(end_speed[:6]), "gripper_end_speed_m_s": end_speed[6],
            "mimic_error_m": mimic_error, "diagnostic_level": node.status_level,
            "diagnostic_text": node.status_text,
        }
        return result


def call(node, service_type, service, request):
    client = node.create_client(service_type, service)
    if not client.wait_for_service(timeout_sec=PLAN["state_check_timeout_sec"]):
        raise RuntimeError(f"没有服务 {service}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=PLAN["controller_timeout_sec"])
    if not future.done() or future.result() is None or not future.result().ok:
        raise RuntimeError(f"失败或超时：{service}")
    return future.result()


def disable_parked(node):
    switch = SwitchController.Request()
    switch.deactivate_controllers = CONTROLLERS
    switch.strictness = SwitchController.Request.STRICT
    switch.timeout.sec = int(CONFIG["safety"]["estop"]["service_timeout_sec"])
    call(node, SwitchController, "/controller_manager/switch_controller", switch)
    request = SetHardwareComponentState.Request()
    request.name = CONFIG["safety"]["estop"]["hardware_component"]
    request.target_state.id = State.PRIMARY_STATE_INACTIVE
    request.target_state.label = "inactive"
    response = call(node, SetHardwareComponentState,
                    "/controller_manager/set_hardware_component_state", request)
    if response.state.id != State.PRIMARY_STATE_INACTIVE:
        raise RuntimeError(f"硬件依然是 {response.state.label}")


def wait_for_inactive(node, timeout=PLAN["controller_timeout_sec"]):
    name = CONFIG["safety"]["estop"]["hardware_component"]
    client = node.create_client(ListHardwareComponents, "/controller_manager/list_hardware_components")
    deadline = time.monotonic() + timeout
    try:
        if not client.wait_for_service(timeout_sec=min(timeout, PLAN["state_check_timeout_sec"])):
            return False
        while (remaining := deadline - time.monotonic()) > 0:
            response = client.call_async(ListHardwareComponents.Request())
            rclpy.spin_until_future_complete(node, response, timeout_sec=remaining)
            if not response.done() or response.result() is None:
                return False
            if any(component.name == name and component.state.id == State.PRIMARY_STATE_INACTIVE
                   for component in response.result().component):
                return True
            rclpy.spin_once(node, timeout_sec=min(PERIOD, max(0.0, deadline - time.monotonic())))
        return False
    finally:
        node.destroy_client(client)


def readonly(node, seconds):
    process = bringup(False)
    try:
        ready(node, process, "readonly")
        q, _ = state(node)
        print("只读反馈:", json.dumps(q), "安全状态:", node.status_text, flush=True)
        start = time.monotonic()
        next_position = start
        with (OUT / "readonly_trace.csv").open("w") as output:
            writer = csv.writer(output)
            writer.writerow(["time_s", *OBSERVED, *[n + "_vel" for n in OBSERVED]])
            while time.monotonic() - start < seconds:
                first = len(node.samples)
                rclpy.spin_once(node, timeout_sec=PLAN["callback_poll_sec"])
                node.check()
                for stamp, q, dq in node.samples[first:]:
                    writer.writerow([stamp - start, *q, *dq])
                if time.monotonic() >= next_position:
                    print("只读实时位置:", [round(v, 4) for v in node.q()], flush=True)
                    next_position += PLAN["joint_state_timeout_sec"]
        q, _ = state(node)
        (OUT / "readonly_pose.json").write_text(json.dumps(q) + "\n")
    finally:
        stop(process)


def score(result, previous):
    result["arm_axes"] = {
        name: {
            "commanded_step_rad": result["target"][index] - previous[index],
            "actual_step_rad": result["actual"][index],
            "steady_error_rad": abs(result["target"][index] - result["after"][index]),
        }
        for index, name in enumerate(ARM)
    }
    return score_hardware_pose(result)


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--readonly", action="store_true")
    mode.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    check_cancelled()
    rclpy.init()
    node = FivePoseHarness()
    try:
        if args.readonly:
            readonly(node, HARDWARE["readonly_duration_sec"])
        else:
            return run(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def run(node):
    validate_configuration(bringup_config('b601_mit_controllers.yaml'))
    probe = json.loads((OUT / "probe.json").read_text())
    initial = json.loads((OUT / "readonly_pose.json").read_text())
    if max(abs(a - b) for a, b in zip(initial[:6], PARK[:6])) > HARDWARE["initial_arm_tolerance_rad"]:
        raise RuntimeError(f"初始手臂超出归中启动范围：{initial[:GRIPPER_INDEX]}")
    if abs(initial[GRIPPER_INDEX] - PARK[GRIPPER_INDEX]) > HARDWARE["initial_gripper_tolerance_m"]:
        raise RuntimeError(f"初始夹爪未接近闭合：{initial[6]:.5f} m")
    if any(probe[name]["firmware_mode"] != 1 for name in NAMES):
        raise RuntimeError("探针未确认全轴 MIT 模式")
    initial = in_range(initial)
    nudge = initial.copy()
    small_index = ARM.index(HARDWARE["small_step_joint"])
    nudge[small_index] += HARDWARE["small_step_rad"]
    paths = [("j1 small motion", initial, nudge), ("return small", nudge, PARK),
             ("approach", PARK, PLAN["start"])]
    paths.extend((f"park from {pose['name']}", pose["target"], PARK) for pose in PLAN["poses"])
    paths.append(("park from start", PLAN["start"], PARK))
    mesh = launch("active_move_group.log", str(COLLISION_LAUNCH))
    process = None
    safe_to_stop = False
    summary = {"results": [], "disabled": False}
    try:
        client = node.create_client(GetStateValidity, "/check_state_validity")
        if not client.wait_for_service(timeout_sec=PLAN["moveit_timeout_sec"]):
            raise RuntimeError("MoveIt 碰撞检查不可用")
        summary["collision_samples"] = check_plan(node, client)
        for label, a, b in paths:
            summary["collision_samples"] += precheck(node, client, label, a, b)
        print("厂商碰撞网格计划采样通过:", summary["collision_samples"], flush=True)
        check_cancelled()
        process = bringup(True)
        ready(node, process, "hardware")
        q, observed = state(node)
        if max(abs(a - b) for a, b in zip(q[:6], PARK[:6])) > HARDWARE["initial_arm_tolerance_rad"]:
            raise RuntimeError(f"使能后起点漂移超限：{q}")
        if max(abs(x) for _, _, v in observed for x in v[:6]) > HARDWARE["estop_arm_speed_rad_s"]:
            raise TrialOverspeed("使能持位期间超过测试速度中止线")
        print(f'6+1 电机侧 MIT 已使能，执行 {HARDWARE["small_step_joint"]} '
              f'{HARDWARE["small_step_rad"]} rad 预检查。', flush=True)
        with (OUT / "trace.csv").open("w", newline="", buffering=1) as output:
            writer = csv.writer(output)
            writer.writerow(["phase", "monotonic_s", *OBSERVED, *[n + "_vel" for n in OBSERVED]])
            motion = Motion(node, writer)
            overspeed = False
            try:
                small = motion.move("small", initial, nudge, HARDWARE["small_slew_rad_s"])
                print("预检查", json.dumps(small), flush=True)
                if (small["actual"][small_index] < HARDWARE["small_min_actual_rad"]
                        or small["arm_max_error_rad"] > HARDWARE["small_max_error_rad"]):
                    raise RuntimeError("小幅预检查未通过")
                motion.move("small_return", nudge, PARK, HARDWARE["small_slew_rad_s"])
                motion.move("approach", PARK, PLAN["start"], HARDWARE["approach_slew_rad_s"])
                previous = PLAN["start"]
                for pose in PLAN["poses"]:
                    target = pose["target"]
                    result = motion.move(pose["name"], previous, target, PLAN["arm_slew_rad_s"])
                    result = score(result, previous)
                    summary["results"].append(result)
                    print(json.dumps(result), flush=True)
                    previous = target
            except TrialCancelled:
                summary["cancelled"] = True
                print("停止测试目标，进入归中失能。", flush=True)
            except TrialOverspeed:
                overspeed = True
                print("试验超速，停止发运动目标，等待独立安全节点判定。", flush=True)
                raise
            finally:
                if not overspeed and node.status_level < 2:
                    motion.recovering = True
                    motion.monitor(HARDWARE["observation_pause_sec"], motion=False)
                    current, _ = state(node)
                    reference = motion.reference or in_range(current)
                    precheck(node, client, "park actual", in_range(current), PARK)
                    precheck(node, client, "park reference", reference, PARK)
                    park = motion.move("park", reference, PARK, HARDWARE["park_slew_rad_s"],
                                       settle=HARDWARE["park_settle_sec"])
                    print("归中", json.dumps(park), flush=True)
                    summary["park"] = park
                    if park["arm_max_error_rad"] > PARK_TOLERANCE:
                        fine_limit = HARDWARE["park_fine_max_rad"]
                        correction = in_range([
                            goal + max(-fine_limit, min(fine_limit, goal - feedback))
                            for goal, feedback in zip(PARK, park["after"])
                        ])
                        precheck(node, client, "park fine adjustment", PARK, correction)
                        fine = motion.move("park_fine", PARK, correction, HARDWARE["park_fine_slew_rad_s"],
                                           settle=HARDWARE["park_fine_settle_sec"])
                        fine["arm_max_park_error_rad"] = max(
                            abs(a - b) for a, b in zip(PARK[:6], fine["after"][:6]))
                        summary["park_fine"] = fine
                        print("归中微调", json.dumps(fine), flush=True)
                        park_error = fine["arm_max_park_error_rad"]
                        park_speed = fine["arm_end_speed_rad_s"]
                    else:
                        park_error = park["arm_max_error_rad"]
                        park_speed = park["arm_end_speed_rad_s"]
                    if park_error <= PARK_TOLERANCE and park_speed <= CRITERIA["arm_end_speed_rad_s"]:
                        disable_parked(node)
                        summary["disabled"] = True
                        safe_to_stop = True
                        print("已确认归中，硬件 INACTIVE。", flush=True)
                    else:
                        raise RuntimeError(f"归中反馈未达 {PARK_TOLERANCE} rad，保留当前控制器持位")
    except RuntimeError as exc:
        if process is not None and (isinstance(exc, TrialOverspeed) or node.status_level >= 2):
            summary["estop_fault"] = str(exc)
            try:
                safe_to_stop = wait_for_inactive(node)
            except Exception as error:
                summary["estop_state_check_error"] = str(error)
            summary["disabled"] = safe_to_stop
            if safe_to_stop:
                print("急停后已独立确认硬件 INACTIVE。", flush=True)
            else:
                print("未确认硬件失能，请使用现场急停并核对驱动状态。", flush=True)
        raise
    finally:
        summary["passed"] = sum(result["passed"] for result in summary["results"])
        summary["arm_passed"] = sum(result["arm_passed"] for result in summary["results"])
        summary["gripper_passed"] = sum(result["gripper_passed"] for result in summary["results"])
        summary["criteria"] = HARDWARE
        (OUT / "results.json").write_text(json.dumps(summary, indent=2) + "\n")
        if process is not None:
            if safe_to_stop:
                stop(process)
            else:
                print(f"真机进程 PID {process.pid} 保留，先核实安全节点及硬件状态。", flush=True)
        stop(mesh)
    return 0 if summary["disabled"] and summary["passed"] == len(PLAN["poses"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
