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
from sensor_msgs.msg import JointState

from mit_five_poses import (
    CONFIG, CRITERIA, NAMES, OBSERVED, PLAN, FivePoseHarness,
    check_plan, checked_sample, score_hardware_pose, validate_configuration,
)

from b601_demos.mit_recenter_client import request_recenter
from b601_demos.trajectory_utils import RECENTER_POSITIONS, at_recenter

from project_config import ROOT

HARDWARE = PLAN["hardware_retest"]
OUT = ROOT / HARDWARE["output_dir"]
HOLD_FILE = OUT / "HOLD_HARDWARE"
COLLISION_LAUNCH = ROOT / "scripts/mit_collision_meshes.launch.py"

CONTROLLERS = ["joint_state_broadcaster", "b601_mit_arm", "b601_mit_gripper"]
ARM = CONFIG["robot"]["arm_joints"]
GRIPPER = CONFIG["robot"]["gripper_joint"]
GRIPPER_INDEX = len(ARM)
PARK = [*RECENTER_POSITIONS, HARDWARE["gripper_parking_target_m"]]
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
    check_collisions = str(HARDWARE["recenter_check_collisions"]).lower()
    return launch(f"{mode}_bringup.log", "b601_bringup", "bringup.launch.py",
                  "hardware:=dm", f"dm_enable_motors:={str(enable).lower()}",
                  "dm_command_mode:=mit_impedance", "use_rviz:=false", "use_safety:=true",
                  f"mit_recenter_check_collisions:={check_collisions}",
                  f"controllers_file:={bringup_config('b601_mit_controllers.yaml')}",
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


class Motion:
    def __init__(self, node, writer):
        self.node = node
        self.writer = writer
        self.recovering = False

    def monitor(self, duration):
        node = self.node
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if not self.recovering:
                check_cancelled()
            rclpy.spin_once(node, timeout_sec=min(PERIOD, max(0.0, end - time.monotonic())))
            node.check()

    def move(self, label, start, goal, settle=PLAN["settle_sec"]):
        if not self.recovering:
            check_cancelled()
        node = self.node
        before, _ = state(node)
        delta = [b - a for a, b in zip(start, goal)]
        steps = max(HARDWARE["minimum_motion_samples"], math.ceil(1.875 / PERIOD * max(
            max(map(abs, delta[:6])) / CONFIG["target_slew"]["revolute"],
            abs(delta[GRIPPER_INDEX]) / CONFIG["target_slew"]["prismatic"])))
        tick = time.monotonic()
        index = len(node.samples)
        try:
            for step in range(1, steps + 1):
                if not self.recovering:
                    check_cancelled()
                u = step / steps
                blend = u**3 * (10 + u * (-15 + 6 * u))
                reference = goal if step == steps else [a + blend * d for a, d in zip(start, delta)]
                node.command(reference)
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
    if not at_recenter(initial[:GRIPPER_INDEX], mit=True):
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
        q, _ = state(node)
        if not at_recenter(q[:GRIPPER_INDEX], mit=True):
            raise RuntimeError(f"使能后起点漂移超限：{q}")
        print(f'6+1 电机侧 MIT 已使能，执行 {HARDWARE["small_step_joint"]} '
              f'{HARDWARE["small_step_rad"]} rad 预检查。', flush=True)
        with (OUT / "trace.csv").open("w", newline="", buffering=1) as output:
            writer = csv.writer(output)
            writer.writerow(["phase", "monotonic_s", *OBSERVED, *[n + "_vel" for n in OBSERVED]])
            motion = Motion(node, writer)
            try:
                small = motion.move("small", initial, nudge)
                print("预检查", json.dumps(small), flush=True)
                if (small["actual"][small_index] < HARDWARE["small_min_actual_rad"]
                        or small["arm_max_error_rad"] > HARDWARE["small_max_error_rad"]):
                    raise RuntimeError("小幅预检查未通过")
                motion.move("small_return", nudge, PARK)
                motion.move("approach", PARK, PLAN["start"])
                previous = PLAN["start"]
                for pose in PLAN["poses"]:
                    target = pose["target"]
                    result = motion.move(pose["name"], previous, target)
                    result = score(result, previous)
                    summary["results"].append(result)
                    print(json.dumps(result), flush=True)
                    previous = target
            except TrialCancelled:
                summary["cancelled"] = True
                print("停止测试目标，进入归中失能。", flush=True)
            finally:
                if node.status_level < 2:
                    motion.recovering = True
                    motion.monitor(HARDWARE["observation_pause_sec"])
                    index = len(node.samples)
                    try:
                        request_recenter(node, CONFIG)
                        current, _ = state(node)
                        grip_target = [*current[:GRIPPER_INDEX], PARK[GRIPPER_INDEX]]
                        precheck(node, client, "park gripper", current, grip_target)
                        node.gripper.publish(JointState(name=[GRIPPER], position=grip_target[-1:]))
                        motion.monitor(HARDWARE["park_settle_sec"])
                        after, final_rows = state(node, PLAN["settle_window_sec"])
                    finally:
                        for stamp, q, dq in node.samples[index:]:
                            writer.writerow(["park", stamp, *q, *dq])
                    park = {
                        "after": after,
                        "arm_max_error_rad": max(abs(actual - goal)
                                                 for actual, goal in zip(after, PARK[:GRIPPER_INDEX])),
                        "arm_end_speed_rad_s": max(
                            sum(abs(row[2][i]) for row in final_rows) / len(final_rows)
                            for i in range(GRIPPER_INDEX)),
                    }
                    print("归中", json.dumps(park), flush=True)
                    summary["park"] = park
                    if (at_recenter(after[:GRIPPER_INDEX], mit=True)
                            and park["arm_end_speed_rad_s"] <= CRITERIA["arm_end_speed_rad_s"]):
                        disable_parked(node)
                        summary["disabled"] = True
                        safe_to_stop = True
                        print("已确认归中，硬件 INACTIVE。", flush=True)
                    else:
                        raise RuntimeError("归中后位置或停稳复核未通过，保持电机使能")
    except RuntimeError as exc:
        if process is not None and node.status_level >= 2:
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
