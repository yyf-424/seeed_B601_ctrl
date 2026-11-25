#!/usr/bin/env python3
"""Start an isolated MuJoCo MIT fixture for integration checks or web preview."""
import argparse
import json
import os
import re
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

import rclpy
from rclpy.executors import ExternalShutdownException
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from b601_description.project_config import load_project_config
from b601_demos.target_arbiter import ARBITER
from b601_demos.target_client import TargetClient
from b601_target_interfaces.msg import TargetFrame
from b601_target_interfaces.srv import RevokeSource, SelectSource, SetTarget

ARM = "b601_mit_arm"
GRIPPER = "b601_mit_gripper"
TIMEOUT = 8.0


class TestNode(Node):
    def __init__(self):
        super().__init__("target_arbitration_integration")
        self.feedback = {}
        self.status = {}
        self.safety = None
        self.holds = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, f"{ARBITER}/status", self.on_status, latched)
        self.create_subscription(JointState, "/joint_states", self.on_feedback, qos_profile_sensor_data)
        self.create_subscription(DiagnosticStatus, "/b601_safety/status", self.on_safety, latched)
        for controller in (ARM, GRIPPER):
            self.create_subscription(
                Bool, f"/{controller}/target_holding",
                lambda msg, controller=controller: self.on_hold(controller, msg), 10)

    def on_status(self, message):
        self.status = json.loads(message.data)

    def on_feedback(self, message):
        self.feedback = dict(zip(message.name, message.position))

    def on_hold(self, controller, message):
        self.holds[controller] = message.data

    @property
    def watchdog_hold(self):
        return self.holds.get(ARM, False)

    def on_safety(self, message):
        self.safety = message.level

    def controller(self, controller=ARM):
        return self.status.get("controllers", {}).get(controller, {})

    def until(self, condition, label, timeout=TIMEOUT):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not condition():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not condition():
            raise AssertionError(f"{label}; status={self.status}; feedback={self.feedback}; safety={self.safety}")

    def applied(self, dispatch_id, controller=ARM):
        self.until(lambda: self.controller(controller).get("last_applied_id") == str(dispatch_id),
                   f"{controller} must acknowledge dispatch {dispatch_id}")

    def reaches(self, values, tolerance=0.004):
        self.until(
            lambda: all(joint in self.feedback and abs(self.feedback[joint] - goal) < tolerance
                        for joint, goal in values.items()),
            f"MuJoCo must reach {values}",
        )


def test(node, launch_log):
    external = TargetClient(node, "external")
    policy = TargetClient(node, "policy")
    gripper = TargetClient(node, "external_gripper")
    node.until(lambda: node.controller().get("joints") and len(node.feedback) == 8 and node.safety is not None
               and len(node.holds) == 2
               and all(node.controller(name).get("feedback_ready") for name in (ARM, GRIPPER)),
               "MIT controllers, live feedback and safety must be present")
    for controller in (ARM, GRIPPER):
        pubs = node.get_publishers_info_by_topic(f"/{controller}/authorized_target")
        assert [info.node_name for info in pubs] == ["b601_target_arbiter"], pubs
        assert node.count_subscribers(f"/{controller}/joint_target") == 0
        assert node.controller(controller)["source"] is None
    print("PASS idle startup and one publisher per controller")

    first_frame = TargetFrame(session_id="no-session", sequence=1,
                              target=JointState(name=["joint1"], position=[-0.1]))
    request = SetTarget.Request(frame=first_frame)
    assert not external.call(external.sender, request, TIMEOUT).accepted
    external.select([ARM], TIMEOUT)
    gripper.select([GRIPPER], TIMEOUT)
    try:
        busy = policy.call(policy.selector, SelectSource.Request(source="policy", controllers=[ARM]), TIMEOUT)
        assert not busy.accepted and "external" in busy.message
        contender = policy.call(policy.selector, SelectSource.Request(
            source="another_external", controllers=[ARM, GRIPPER]), TIMEOUT)
        assert not contender.accepted and "external_gripper" in contender.message
        wrong_group = external.frame(JointState(name=["finger_left"], position=[0.02]))
        assert not external.call(external.sender, SetTarget.Request(frame=wrong_group), TIMEOUT).accepted
        gripper_goal = node.feedback["finger_left"] + 0.0003
        grip = gripper.set_joint("finger_left", gripper_goal, TIMEOUT)
        node.applied(grip, GRIPPER)
        node.reaches({"finger_left": gripper_goal}, tolerance=0.00015)
        print("PASS exclusive claims and independent simulated gripper")

        old = external.frame(JointState(name=["joint1"], position=[0.12]))
        response = external.call(external.sender, SetTarget.Request(frame=old), TIMEOUT)
        assert response.accepted, response.message
        node.applied(response.dispatch_id)
        node.reaches({"joint1": 0.12})
        assert not external.call(external.sender, SetTarget.Request(frame=old), TIMEOUT).accepted
        multi = {"joint1": 0.07, "joint4": 0.05}
        forwarded = external.set_joints(multi, TIMEOUT)
        node.applied(forwarded)
        node.reaches(multi)
        assert node.controller()["last_forwarded"]["position"]["joint1"] == 0.07
        assert node.controller()["last_forwarded"]["position"]["joint4"] == 0.05
        assert node.controller()["last_forwarded"]["source"] == "external"
        print("PASS external single/multi goals, applied IDs, and sequence replay rejection")

        external.release(TIMEOUT)
        policy.select([ARM], TIMEOUT)
        old.sequence += 1000
        old.target.position = [-0.14]
        external.publisher.publish(old)
        assert not external.call(external.sender, SetTarget.Request(frame=old), TIMEOUT).accepted
        node.until(lambda: node.controller()["source"] == "policy", "policy is the selected owner")
        assert node.controller()["last_forwarded"]["dispatch_id"] == str(forwarded)
        new_goal = JointState(name=["joint1", "joint4"], position=[0.02, 0.03], velocity=[0.0, 0.0])
        policy.publish(new_goal)
        node.until(lambda: node.controller()["last_forwarded"]["source"] == "policy",
                   "policy stream must reach the arbiter")
        node.applied(node.controller()["last_forwarded"]["dispatch_id"])
        node.reaches(dict(zip(new_goal.name, new_goal.position)))
        assert node.safety == DiagnosticStatus.OK, node.safety
        print("PASS handover, delayed previous-session rejection and streaming policy")
    finally:
        external.close(TIMEOUT)
        policy.close(TIMEOUT)
        gripper.close(TIMEOUT)

    external.select([ARM, GRIPPER], TIMEOUT)
    try:
        combined_goal = {"joint1": 0.04, "finger_left": gripper_goal + 0.0002}
        combined = external.set_joints(combined_goal, TIMEOUT)
        node.applied(combined)
        node.applied(combined, GRIPPER)
        node.reaches({"joint1": combined_goal["joint1"]})
        node.reaches({"finger_left": combined_goal["finger_left"]}, tolerance=0.00015)
        print("PASS one multi-group target reaches both MuJoCo controllers")
    finally:
        external.close(TIMEOUT)

    recenter = node.create_client(Trigger, "/b601_mit_recenter/recenter")
    assert recenter.wait_for_service(timeout_sec=TIMEOUT)
    result = recenter.call_async(Trigger.Request())
    node.until(lambda: node.controller().get("source") == "recenter", "recenter must acquire ownership")
    contention = policy.call(
        policy.selector, SelectSource.Request(source="policy", controllers=[ARM]), TIMEOUT)
    assert not contention.accepted and "recenter" in contention.message
    node.until(result.done, "recenter action must finish", timeout=45)
    assert result.result().success, result.result().message
    node.until(lambda: node.controller()["source"] is None, "recenter must release ownership")
    assert node.controller()["last_forwarded"]["source"] == "recenter"
    project = load_project_config()
    for joint, value in project["recenter"]["joints"].items():
        if joint in node.controller()["joints"]:
            assert abs(node.feedback[joint] - value) < 0.03, (joint, node.feedback[joint], value)
    assert node.safety in (DiagnosticStatus.OK, DiagnosticStatus.WARN), node.safety
    print("PASS real MuJoCo recenter through arbitration, contention and release")

    revoke = node.create_client(RevokeSource, f"{ARBITER}/revoke_source")
    abandoned = external.call(external.selector, SelectSource.Request(source="abandoned", controllers=[ARM]), TIMEOUT)
    assert abandoned.accepted
    node.until(lambda: node.controller().get("source") == "abandoned", "operator must see orphaned ownership")
    public_id = node.controller()["session"]
    wrong = RevokeSource.Request(controller=ARM, session="old-selection")
    assert not external.call(revoke, wrong, TIMEOUT).accepted
    current = RevokeSource.Request(controller=ARM, session=public_id)
    assert external.call(revoke, current, TIMEOUT).accepted
    print("PASS operator revocation matches only the selected session")

    lost = TargetClient(node, "lost_source", auto_renew=False)
    lost.select([ARM], TIMEOUT)
    lost.renew(TIMEOUT)
    started = time.monotonic()
    first = lost.set_joint("joint1", 0.8, TIMEOUT)
    node.applied(first)
    previous = lost.frame(JointState(name=["joint1"], position=[0.7]))
    node.until(lambda: node.controller().get("source") is None and node.watchdog_hold,
               "lost source must release authority and put the MIT controller into measured hold", timeout=2)
    source_hold_time = time.monotonic() - started
    assert 0.45 <= source_hold_time <= 0.8, source_hold_time
    hold_position = node.feedback["joint1"]
    assert hold_position < 0.4, hold_position
    assert not external.call(external.sender, SetTarget.Request(frame=previous), TIMEOUT).accepted
    node.until(lambda: node.controller()["last_applied_id"] is None, "hold clears prior receipt")
    print(f"PASS source disconnect, hold after {source_hold_time:.3f}s, prior frame rejected")

    policy.select([ARM], TIMEOUT)
    try:
        dispatch = policy.set_joint("joint1", 0.8, TIMEOUT)
        node.applied(dispatch)
        node.until(lambda: not node.watchdog_hold, "fresh selection must leave measured hold")
        log = launch_log.read_text()
        match = re.search(r"target_arbiter-\d+\]: process started with pid \[([0-9]+)\]", log)
        assert match, "arbiter PID not present in this MuJoCo launch"
        arbiter_pid = int(match.group(1))
        started = time.monotonic()
        os.kill(arbiter_pid, signal.SIGSTOP)
        try:
            node.until(lambda: node.watchdog_hold and node.holds.get(GRIPPER),
                       "both controllers must detect missing arbiter independently",
                       timeout=2)
            arbiter_hold_time = time.monotonic() - started
            assert 0.35 <= arbiter_hold_time <= 0.8, arbiter_hold_time
            assert node.feedback["joint1"] < 0.7
        finally:
            os.kill(arbiter_pid, signal.SIGCONT)

        old_frame = policy.frame(JointState(name=["joint1"], position=[0.8]))
        assert not external.call(external.sender, SetTarget.Request(frame=old_frame), TIMEOUT).accepted
        try:
            policy.release(TIMEOUT)
        except RuntimeError as error:
            assert "released session" in str(error)
        node.until(lambda: node.controller().get("source") is None,
                   "resumed arbiter must leave the old owner revoked")
        policy.select([ARM], TIMEOUT)
        recovered = policy.set_joint("joint1", 0.07, TIMEOUT)
        node.applied(recovered)
        node.reaches({"joint1": 0.07})
        node.until(lambda: not node.watchdog_hold, "fresh selection must leave measured hold")
        print(f"PASS arbiter interrupt, hold after {arbiter_hold_time:.3f}s, and new explicit selection")
    finally:
        policy.close(TIMEOUT)
    print(json.dumps({"feedback": node.feedback, "status": node.status}, ensure_ascii=False))


def start_fixture(output, web):
    root = Path(__file__).resolve().parent.parent
    source = root / "ros2_ws/src"
    model = ET.parse(source / "b601_mujoco/models/b601_arm.xml")
    model.getroot().find("option").set("gravity", "0 0 0")
    model_path = output / "model.xml"
    model.write(model_path)

    initial = {
        "initial_positions": {"joint1": 0.0, "joint2": -0.3, "joint3": -0.6,
                              "joint4": 0.0, "joint5": 0.0, "joint6": 0.0,
                              "finger_left": 0.015, "finger_right": -0.015},
    }
    initial_path = output / "initial.yaml"
    initial_path.write_text(yaml.safe_dump(initial))
    config = yaml.safe_load((source / "b601_bringup/config/b601_mit_controllers.yaml").read_text())
    # The committed DM gains overdrive this MuJoCo finger model. Tame only this test fixture.
    finger = config[GRIPPER]["ros__parameters"]
    for key, value in (("kp_hold", 500.0), ("kp_move", 500.0),
                       ("kd_hold", 5.0), ("kd_move", 5.0)):
        finger[key] = [value]
    controllers = output / "controllers.yaml"
    controllers.write_text(yaml.safe_dump(config))
    with (output / "launch.log").open("w") as stream:
        return subprocess.Popen([
            "ros2", "launch", "b601_bringup", "bringup.launch.py",
            "hardware:=mujoco", "dm_command_mode:=mit_impedance", "use_rviz:=false",
            "mit_recenter_check_collisions:=false", f"mujoco_model:={model_path}",
            f"initial_positions_file:={initial_path}", f"controllers_file:={controllers}",
            f"use_target_web:={str(web).lower()}",
        ], stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--web", action="store_true", help="启动同一 MuJoCo 夹具并预览本地网页")
    args = parser.parse_args()
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    domain = os.environ.get("B601_TEST_ROS_DOMAIN_ID", "227")
    if not domain.isdecimal() or not 0 <= int(domain) < 233:
        raise SystemExit("B601_TEST_ROS_DOMAIN_ID must be a ROS domain number.")
    os.environ["ROS_DOMAIN_ID"] = domain
    output = Path(__file__).resolve().parent.parent / ".verify" / f"target-arbitration-{domain}"
    output.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(output / "roslog")
    rclpy.init()
    node = TestNode()
    process = None
    try:
        wait_until = time.monotonic() + 1.0
        while time.monotonic() < wait_until:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.count_publishers("/joint_states") or "controller_manager" in node.get_node_names():
            raise RuntimeError("Test domain already contains a robot, choose B601_TEST_ROS_DOMAIN_ID.")
        process = start_fixture(output, args.web)
        if args.web:
            node.until(lambda: node.controller().get("joints") and bool(node.feedback),
                       "MuJoCo preview must become ready")
            print("MuJoCo：访问 http://127.0.0.1:8765/ ，Ctrl-C 停止。", flush=True)
            while rclpy.ok() and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.1)
            if process.poll() not in (None, 0):
                raise RuntimeError(f"MuJoCo launch failed, see {output / 'launch.log'}")
        else:
            test(node, output / "launch.log")
    except (KeyboardInterrupt, ExternalShutdownException):
        if not args.web:
            raise
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=15)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
