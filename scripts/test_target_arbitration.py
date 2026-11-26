#!/usr/bin/env python3
"""Start an isolated MuJoCo MIT fixture for integration checks or web preview."""
import argparse
import json
import os
import re
import signal
import subprocess
import time
from pathlib import Path

import yaml

import rclpy
from rclpy.executors import ExternalShutdownException
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from b601_description.project_config import load_project_config
from b601_demos.target_arbiter import ARBITER
from b601_demos.target_client import TargetClient
from b601_target_interfaces.msg import TargetFrame
from b601_target_interfaces.srv import RevokeSource, SelectSource, SetAuthority, SetTarget

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

    def until(self, condition, label, timeout=TIMEOUT):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not condition():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not condition():
            raise AssertionError(f"{label}; status={self.status}; feedback={self.feedback}; safety={self.safety}")

    def applied(self, dispatch_id):
        self.until(lambda: self.status.get("last_applied_id") == str(dispatch_id),
                   f"all routes must acknowledge dispatch {dispatch_id}")

    def reaches(self, values, tolerance=0.004):
        self.until(
            lambda: all(joint in self.feedback and abs(self.feedback[joint] - goal) < tolerance
                        for joint, goal in values.items()),
            f"MuJoCo must reach {values}",
        )


def test(node, launch_log):
    external = TargetClient(node, "external")
    policy = TargetClient(node, "policy")
    authority = node.create_client(SetAuthority, f"{ARBITER}/set_authority")
    node.until(lambda: node.status.get("feedback_ready") and len(node.feedback) == 8
               and node.safety is not None and len(node.holds) == 2,
               "MIT controllers, live feedback and safety must be present")
    assert len(node.status["joints"]) == 7 and "controllers" not in node.status
    assert node.status["source"] is None and node.status["authority"] is None
    for controller in (ARM, GRIPPER):
        pubs = node.get_publishers_info_by_topic(f"/{controller}/authorized_target")
        assert [info.node_name for info in pubs] == ["b601_target_arbiter"], pubs
        inputs = node.get_publishers_info_by_topic(f"/{controller}/joint_target")
        assert [info.node_name for info in inputs] == ["b601_target_bridge"], inputs
        subscribers = node.get_subscriptions_info_by_topic(f"/{controller}/joint_target")
        assert [info.node_name for info in subscribers] == [controller], subscribers
    print("PASS idle startup and one publisher per controller")

    first_frame = TargetFrame(session_id="no-session", sequence=1,
                              target=JointState(name=["joint1"], position=[-0.1]))
    assert not external.call(external.sender, SetTarget.Request(frame=first_frame), TIMEOUT).accepted
    external.select(TIMEOUT)
    try:
        busy = policy.call(policy.selector, SelectSource.Request(source="policy"), TIMEOUT)
        assert not busy.accepted and "external" in busy.message
        partial = external.frame(JointState(name=["finger_left"], position=[0.02]))
        rejected = external.call(external.sender, SetTarget.Request(frame=partial), TIMEOUT)
        assert not rejected.accepted and "all seven" in rejected.message

        positions = {name: node.feedback[name] for name in node.status["joints"]}
        goal = {"joint1": 0.12, "finger_left": positions["finger_left"] + 0.0003}
        positions.update(goal)
        old = external.frame(JointState(name=list(positions), position=list(positions.values())))
        response = external.call(external.sender, SetTarget.Request(frame=old), TIMEOUT)
        assert response.accepted, response.message
        node.applied(response.dispatch_id)
        node.reaches({"joint1": goal["joint1"]})
        node.reaches({"finger_left": goal["finger_left"]}, tolerance=0.00015)
        assert not external.call(external.sender, SetTarget.Request(frame=old), TIMEOUT).accepted
        assert node.status["last_forwarded"]["position"] == positions
        print("PASS exclusive owner, full seven-joint wire target, replay rejection and both routes")

        reply = external.call(authority, SetAuthority.Request(source="policy"), TIMEOUT)
        assert reply.accepted
        node.until(lambda: node.status.get("authority") == "policy" and node.status.get("source") is None,
                   "designating a new input must revoke the prior holder")
        denied = external.call(external.selector, SelectSource.Request(source="external"), TIMEOUT)
        assert not denied.accepted and "designated to policy" in denied.message
        policy.select(TIMEOUT)
        assert external.call(authority, SetAuthority.Request(source="policy"), TIMEOUT).accepted
        old.sequence += 1000
        old.target.position[0] = -0.14
        external.publisher.publish(old)
        assert not external.call(external.sender, SetTarget.Request(frame=old), TIMEOUT).accepted
        node.until(lambda: node.status["source"] == "policy", "policy is the designated owner")
        assert node.status["last_forwarded"]["dispatch_id"] == str(response.dispatch_id)
        new_goal = JointState(name=["joint1", "joint4"], position=[0.02, 0.03], velocity=[0.0, 0.0])
        policy.publish(new_goal)
        node.until(lambda: node.status["last_forwarded"]["source"] == "policy",
                   "policy stream must reach the arbiter")
        node.applied(node.status["last_forwarded"]["dispatch_id"])
        node.reaches(dict(zip(new_goal.name, new_goal.position)))
        assert list(node.status["last_forwarded"]["position"]) == node.status["joints"]
        assert node.safety == DiagnosticStatus.OK, node.safety

        policy.release(TIMEOUT)
        denied = external.call(external.selector, SelectSource.Request(source="external"), TIMEOUT)
        assert not denied.accepted and "designated to policy" in denied.message
        policy.select(TIMEOUT)
        assert external.call(authority, SetAuthority.Request(source=""), TIMEOUT).accepted
        node.until(lambda: node.status["source"] is None and node.status["authority"] is None,
                   "operator release must clear the designation and the active holder")
        print("PASS operator handover, previous-session rejection, policy stream and release")
    finally:
        external.close(TIMEOUT)
        policy.close(TIMEOUT)

    reserve_disable = node.create_client(SetBool, f"{ARBITER}/reserve_disable")
    assert not external.call(reserve_disable, SetBool.Request(data=True), TIMEOUT).success

    recenter = node.create_client(Trigger, "/b601_mit_recenter/recenter")
    assert recenter.wait_for_service(timeout_sec=TIMEOUT)
    result = recenter.call_async(Trigger.Request())
    node.until(lambda: node.status.get("source") == "recenter", "recenter must acquire ownership")
    contention = policy.call(
        policy.selector, SelectSource.Request(source="policy"), TIMEOUT)
    assert not contention.accepted and "recenter" in contention.message
    node.until(result.done, "recenter action must finish", timeout=45)
    assert result.result().success, result.result().message
    node.until(lambda: node.status["source"] is None, "recenter must release ownership")
    assert node.status["last_forwarded"]["source"] == "recenter"
    project = load_project_config()
    for joint, value in project["recenter"]["joints"].items():
        if joint in node.status["joints"]:
            assert abs(node.feedback[joint] - value) < 0.03, (joint, node.feedback[joint], value)
    assert node.safety in (DiagnosticStatus.OK, DiagnosticStatus.WARN), node.safety
    node.until(lambda: node.status.get("disable_ready"), "successful recenter must unlock disable")
    assert external.call(reserve_disable, SetBool.Request(data=True), TIMEOUT).success
    try:
        assert not external.call(policy.selector, SelectSource.Request(source="policy"), TIMEOUT).accepted
        assert not external.call(authority, SetAuthority.Request(source="policy"), TIMEOUT).accepted
    finally:
        assert external.call(reserve_disable, SetBool.Request(data=False), TIMEOUT).success
    print("PASS MuJoCo recenter and shutdown gate block competing control sources")

    revoke = node.create_client(RevokeSource, f"{ARBITER}/revoke_source")
    abandoned = external.call(external.selector, SelectSource.Request(source="abandoned"), TIMEOUT)
    assert abandoned.accepted
    node.until(lambda: node.status.get("source") == "abandoned", "operator must see orphaned ownership")
    public_id = node.status["session"]
    wrong = RevokeSource.Request(session="old-selection")
    assert not external.call(revoke, wrong, TIMEOUT).accepted
    current = RevokeSource.Request(session=public_id)
    assert external.call(revoke, current, TIMEOUT).accepted
    print("PASS operator revocation matches only the selected session")

    lost = TargetClient(node, "lost_source", auto_renew=False)
    lost.select(TIMEOUT)
    lost.renew(TIMEOUT)
    started = time.monotonic()
    first = lost.set_joint("joint1", 0.8, TIMEOUT)
    node.applied(first)
    previous = lost.frame(JointState(name=["joint1"], position=[0.7]))
    node.until(lambda: node.status.get("source") is None and node.watchdog_hold and node.holds.get(GRIPPER),
               "lost source must release authority and put the MIT controller into measured hold", timeout=2)
    source_hold_time = time.monotonic() - started
    assert 0.45 <= source_hold_time <= 0.8, source_hold_time
    hold_position = node.feedback["joint1"]
    assert hold_position < 0.4, hold_position
    assert not external.call(external.sender, SetTarget.Request(frame=previous), TIMEOUT).accepted
    node.until(lambda: node.status["last_applied_id"] is None, "hold clears prior receipt")
    print(f"PASS source disconnect, hold after {source_hold_time:.3f}s, prior frame rejected")

    policy.select(TIMEOUT)
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
                       "the bridge must forward a measured hold to both controllers",
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
        node.until(lambda: node.status.get("source") is None,
                   "resumed arbiter must leave the old owner revoked")
        policy.select(TIMEOUT)
        recovered = policy.set_joint("joint1", 0.07, TIMEOUT)
        node.applied(recovered)
        node.reaches({"joint1": 0.07})
        node.until(lambda: not node.watchdog_hold, "fresh selection must leave measured hold")
        print(f"PASS arbiter interrupt, hold after {arbiter_hold_time:.3f}s, and new explicit selection")
    finally:
        policy.close(TIMEOUT)
    print(json.dumps({"feedback": node.feedback, "status": node.status}, ensure_ascii=False))


def test_physical_recenter(node):
    project = load_project_config()
    arm = project["robot"]["arm_joints"]
    home = {name: project["recenter"]["joints"][name] for name in arm}
    recenter = node.create_client(Trigger, "/b601_mit_recenter/recenter")
    reserve = node.create_client(SetBool, f"{ARBITER}/reserve_disable")
    target = TargetClient(node, "external")
    node.until(lambda: node.status.get("feedback_ready") and recenter.service_is_ready(),
               "physical MuJoCo must provide feedback and recenter service")
    for goal in (
        dict(zip(arm, (-0.28, -0.18, -0.38, 0.24, 0.23, 0.0))),
        dict(zip(arm, (0.0, -0.03, -0.16, -0.18, 0.0, 0.0))),
    ):
        target.select(TIMEOUT)
        try:
            node.applied(target.set_joints(goal, TIMEOUT))
            node.reaches(goal)
        finally:
            target.release(TIMEOUT)
        assert not node.status["disable_ready"], node.status
        start = {name: node.feedback[name] for name in arm}
        result = recenter.call_async(Trigger.Request())
        node.until(result.done, "physical MuJoCo recenter must finish", timeout=45)
        assert result.result().success, (start, result.result().message)
        node.reaches(home)
        node.until(lambda: node.status.get("disable_ready"),
                   "physical MuJoCo parking feedback must unlock disable")
        assert target.call(reserve, SetBool.Request(data=True), TIMEOUT).success
        assert target.call(reserve, SetBool.Request(data=False), TIMEOUT).success
        print(f"PASS physical MuJoCo recenter and disable gate from {start}", flush=True)


def start_fixture(output, web, physical=False):
    if physical:
        with (output / "launch.log").open("w") as stream:
            return subprocess.Popen([
                "ros2", "launch", "b601_bringup", "bringup.launch.py",
                "hardware:=mujoco", "mujoco_scene:=physical",
                "dm_command_mode:=mit_impedance", "use_rviz:=false",
            ], stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)

    root = Path(__file__).resolve().parent.parent
    source = root / "ros2_ws/src"

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
            "mit_recenter_check_collisions:=false",
            f"initial_positions_file:={initial_path}", f"controllers_file:={controllers}",
            f"use_target_web:={str(web).lower()}",
        ], stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--web", action="store_true", help="启动同一 MuJoCo 夹具并预览本地网页")
    mode.add_argument("--physical", action="store_true", help="验证内置物理场景的归中及失能门控")
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
        process = start_fixture(output, args.web, args.physical)
        if args.web:
            node.until(lambda: node.status.get("feedback_ready"),
                       "MuJoCo preview must become ready")
            print("MuJoCo：访问 http://127.0.0.1:8765/ ，Ctrl-C 停止。", flush=True)
            while rclpy.ok() and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.1)
            if process.poll() not in (None, 0):
                raise RuntimeError(f"MuJoCo launch failed, see {output / 'launch.log'}")
        elif args.physical:
            test_physical_recenter(node)
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
