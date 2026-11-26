#!/usr/bin/env python3
"""回放 6+1 电机侧 MIT 仿真或按统一配置重评分真机结果。"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from moveit_msgs.srv import GetStateValidity
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import yaml

from project_config import ROOT, load_project_config
from sim_mit_ros import Harness, node_wait_controllers

CONFIG = load_project_config()
PLAN = CONFIG["validation"]["mit_five_poses"]
ROBOT = CONFIG["robot"]
NAMES = [*ROBOT["arm_joints"], ROBOT["gripper_joint"]]
OBSERVED = [*NAMES, ROBOT["mimic_joint"]]
CRITERIA = PLAN["criteria"]


class FivePoseHarness(Harness):
    def __init__(self, source="sim_five_poses"):
        super().__init__("b601_mit_arm", OBSERVED, source)
        self.gripper_ff = []
        self.create_subscription(
            Float64MultiArray, "/b601_mit_gripper/ff", self.on_gripper_ff, 10)

    def on_gripper_ff(self, message):
        self.gripper_ff = list(message.data)

    def command(self, values):
        self.targets.publish(JointState(name=NAMES, position=values))

    def check(self):
        if self.status_level not in (0, 1):
            raise RuntimeError(f"安全诊断异常 {self.status_level}: {self.status_text}")
        if not self.samples or time.monotonic() - self.samples[-1][0] > PLAN["feedback_timeout_sec"]:
            raise RuntimeError("关节反馈断流")
        _, positions, velocities = self.samples[-1]
        if not all(math.isfinite(n) for n in positions + velocities):
            raise RuntimeError("反馈含非有限值")
        if any(abs(x) > 1e-9 for x in self.ff + self.gripper_ff):
            raise RuntimeError("纯 PD 配置中出现非零前馈")

    def hold(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=PLAN["callback_poll_sec"])
            self.check()

    def average(self, rows):
        return [sum(column) / len(column) for column in zip(*(q for _, q, _ in rows))]


def validate_configuration(controllers_file=None):
    if controllers_file is None:
        controllers_file = ROOT / "ros2_ws/install/b601_bringup/share/b601_bringup/config/b601_mit_controllers.yaml"
    tool_cfg = yaml.safe_load(controllers_file.read_text())
    for group, joints in (("b601_mit_arm", NAMES[:6]), ("b601_mit_gripper", NAMES[6:])):
        params = tool_cfg[group]["ros__parameters"]
        assert params["joints"] == joints and params["motor_impedance"] and not params["free_enabled"]
        assert params["gravity_scale"] == params["friction_scale"] == 0
        assert not params["use_gravity"]
    prior = PLAN["start"]
    for pose in PLAN["poses"]:
        for i, name in enumerate(NAMES):
            bound = CONFIG["joints"][name]
            target = pose["target"][i]
            assert bound["min_pos"] <= target <= bound["max_pos"], (pose["name"], name, target)
            assert target != prior[i]
        prior = pose["target"]


def checked_sample(node, client, q, label):
    request = GetStateValidity.Request()
    request.robot_state.joint_state = JointState(name=OBSERVED, position=[*q[:7], -q[6]])
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=PLAN["state_check_timeout_sec"])
    if not future.done() or future.result() is None:
        raise RuntimeError("MoveIt /check_state_validity 响应超时")
    result = future.result()
    if not result.valid:
        pairs = [(c.contact_body_1, c.contact_body_2) for c in result.contacts]
        raise RuntimeError(f"{label} 碰撞或状态无效：{pairs}")


def check_plan(node, client):
    count = 0
    start = PLAN["start"]
    sample_count = PLAN["collision_samples_per_segment"]
    for pose in PLAN["poses"]:
        goal = pose["target"]
        for step in range(sample_count):
            u = step / (sample_count - 1)
            sample = [a + (b - a) * u for a, b in zip(start, goal)]
            checked_sample(node, client, sample, f"{pose['name']} 采样 {step + 1}/{sample_count}")
            count += 1
        start = goal
    return count


def score_hardware_pose(result):
    config = PLAN["hardware_retest"]
    if set(result["arm_axes"]) != set(ROBOT["arm_joints"]):
        raise ValueError(f'{result["name"]} 缺少完整的六轴验收数据')
    axes = {}
    for name, axis in result["arm_axes"].items():
        override = config["joint_error_bands"].get(name, {})
        fraction = override.get("step_error_fraction", config["arm_step_error_fraction"])
        minimum = override.get("min_error_band_rad", config["arm_min_error_band_rad"])
        tolerance = max(minimum, fraction * abs(axis["commanded_step_rad"]))
        axes[name] = {
            **axis, "tolerance_rad": tolerance, "passed": axis["steady_error_rad"] <= tolerance,
        }
    arm_passed = (
        all(axis["passed"] for axis in axes.values())
        and result["min_signed_arm_movement_rad"] >= CRITERIA["min_signed_arm_movement_rad"]
        and result["arm_end_speed_rad_s"] <= CRITERIA["arm_end_speed_rad_s"]
    )
    gripper_passed = (
        result["signed_gripper_movement_m"] >= CRITERIA["min_signed_gripper_movement_m"]
        and result["gripper_error_m"] <= config["gripper_max_error_m"]
        and result["gripper_end_speed_m_s"] <= CRITERIA["gripper_end_speed_m_s"]
        and result["mimic_error_m"] <= PLAN["mimic_tolerance_m"]
    )
    return {
        **result, "arm_axes": axes, "arm_passed": arm_passed, "gripper_passed": gripper_passed,
        "passed": arm_passed and gripper_passed,
    }


def score_hardware_results(path):
    source = json.loads(path.read_text())
    results = [score_hardware_pose(result) for result in source["results"]]
    return {
        **source, "source": str(path.resolve()), "results": results,
        "arm_passed": sum(result["arm_passed"] for result in results),
        "gripper_passed": sum(result["gripper_passed"] for result in results),
        "passed": sum(result["passed"] for result in results),
        "criteria": PLAN["hardware_retest"],
    }


def follow_plan(node, writer):
    previous = PLAN["start"]
    node.command(previous)
    node.hold(PLAN["setup_hold_sec"])
    results = []
    for pose in PLAN["poses"]:
        target = pose["target"]
        displacement = [end - start for start, end in zip(previous, target)]
        beginning = node.average(node.samples[-PLAN["steady_sample_count"]:])
        duration = 1.875 * max(
            max(map(abs, displacement[:6])) / CONFIG["target_slew"]["revolute"],
            abs(displacement[6]) / CONFIG["target_slew"]["prismatic"],
        )
        steps = math.ceil(duration * PLAN["publish_rate_hz"])
        tick = time.monotonic()
        sample_start = len(node.samples)
        for step in range(1, steps + 1):
            u = step / steps
            blend = u**3 * (10 + u * (-15 + 6 * u))
            terminal = step == steps
            reference = target if terminal else [
                start + delta * blend for start, delta in zip(previous, displacement)
            ]
            node.command(reference)
            tick += 1 / PLAN["publish_rate_hz"]
            node.hold(max(0.0, tick - time.monotonic()))
        node.hold(PLAN["settle_sec"])
        rows = node.samples[sample_start:]
        final_rows = [row for row in rows if rows[-1][0] - row[0] < PLAN["settle_window_sec"]]
        after = node.average(final_rows)
        actual = [after[i] - beginning[i] for i in range(8)]
        signed = [value * (1 if goal > 0 else -1) for value, goal in zip(actual, displacement)]
        peaks = [max(abs(v[i]) for _, _, v in rows) for i in range(8)]
        end_speeds = [max(abs(v[i]) for _, _, v in final_rows) for i in range(8)]
        mimic_error = max(abs(q[6] + q[7]) for _, q, _ in rows)
        error = [abs(goal - actual) for goal, actual in zip(target, after)]
        arm_count = len(ROBOT["arm_joints"])
        control_error = node.state[5 * arm_count:6 * arm_count]
        report = {
            "pose": pose["name"], "target": target, "position": after,
            "arm_control_error_rad": control_error,
            "actual_movement": actual,
            "min_signed_arm_movement_rad": min(signed[:6]), "signed_gripper_movement_m": signed[6],
            "arm_max_error_rad": max(error[:6]), "gripper_error_m": error[6],
            "arm_peak_speed_rad_s": max(peaks[:6]), "gripper_peak_speed_m_s": peaks[6],
            "mimic_error_m": mimic_error, "arm_end_speed_rad_s": max(end_speeds[:6]),
            "gripper_end_speed_m_s": end_speeds[6], "diagnostic_level": node.status_level,
        }
        for stamp, q, v in rows:
            writer.writerow([pose["name"], stamp - rows[0][0], *q, *v])
        print(json.dumps(report), flush=True)
        if (
            min(signed[:6]) < CRITERIA["min_signed_arm_movement_rad"]
            or signed[6] < CRITERIA["min_signed_gripper_movement_m"]
            or max(error[:6]) > CRITERIA["arm_max_error_rad"]
            or error[6] > CRITERIA["gripper_max_error_m"]
            or max(end_speeds[:6]) > CRITERIA["arm_end_speed_rad_s"]
            or end_speeds[6] > CRITERIA["gripper_end_speed_m_s"]
            or mimic_error > PLAN["mimic_tolerance_m"]
        ):
            raise RuntimeError(f"{pose['name']} 实测运动或停稳未达到验收条件")
        results.append(report)
        previous = target
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / PLAN["output_dir"])
    parser.add_argument("--controllers-file", type=Path, help="仿真候选控制器参数文件")
    parser.add_argument("--score-hardware", type=Path, metavar="RESULTS_JSON",
                        help="按当前分轴验收带重评分已记录的真机结果")
    args = parser.parse_args()
    if args.score_hardware is not None:
        print(json.dumps(score_hardware_results(args.score_hardware), indent=2, ensure_ascii=False))
        return
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("ROS_DOMAIN_ID", str(PLAN["ros_domain_id"]))
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    os.environ.setdefault("RMW_FASTRTPS_PUBLICATION_MODE", "ASYNCHRONOUS")
    validate_configuration(args.controllers_file)
    launch_args = [
        ["b601_bringup", "bringup.launch.py", "hardware:=mujoco",
         "dm_command_mode:=mit_impedance", "use_rviz:=false"],
        ["b601_moveit_config", "move_group.launch.py", "hardware:=mujoco", "use_rviz:=false"],
    ]
    if args.controllers_file is not None:
        launch_args[0].append(f"controllers_file:={args.controllers_file.resolve()}")
    processes = []
    rclpy.init()
    node = FivePoseHarness()
    try:
        def launch(name, arguments):
            with (output / f"{name}.log").open("w") as logfile:
                return subprocess.Popen(
                    ["ros2", "launch", *arguments], stdout=logfile, stderr=subprocess.STDOUT,
                    start_new_session=True)

        controllers = ["joint_state_broadcaster", *CONFIG["controllers"]["mit"]["instances"]]
        for attempt in range(PLAN["startup_attempts"]):
            process = launch(f"bringup_{attempt + 1}", launch_args[0])
            processes.append(process)
            if node_wait_controllers(node, controllers, PLAN["controller_timeout_sec"]):
                break
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=PLAN["shutdown_timeout_sec"])
        else:
            raise RuntimeError("默认 6+1 控制器未激活")
        processes.append(launch("move_group", launch_args[1]))
        service = node.create_client(GetStateValidity, "/check_state_validity")
        if not service.wait_for_service(timeout_sec=PLAN["moveit_timeout_sec"]):
            raise RuntimeError("MoveIt 状态有效性服务未就绪")
        count = check_plan(node, service)
        if not node.wait_ready(PLAN["joint_state_timeout_sec"]):
            raise RuntimeError("缺少关节反馈")
        node.hold(PLAN["forward_observation_sec"])
        if len(node.ff) != 18 or len(node.gripper_ff) != 3:
            raise RuntimeError("6+1 前馈观测未就绪")
        node.targets.select(PLAN["controller_timeout_sec"])
        print(f"MoveIt 计划路径采样通过 {count}/{count}，MuJoCo 开始运动验证。", flush=True)
        with (output / "trace.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["pose", "time_s", *OBSERVED, *[name + "_vel" for name in OBSERVED]])
            results = follow_plan(node, writer)
        summary = {"collision_samples": count, "passed": len(results), "results": results}
        (output / "results.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(f'通过 {len(results)}/{len(PLAN["poses"])}。', flush=True)
    finally:
        node.targets.close(PLAN["controller_timeout_sec"])
        node.destroy_node()
        rclpy.try_shutdown()
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in processes:
            process.wait(timeout=PLAN["shutdown_timeout_sec"])


if __name__ == "__main__":
    main()
