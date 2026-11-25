"""Client for selecting a source and supplying sequenced MIT joint targets."""
from math import isfinite
from time import monotonic
from threading import Event, Lock

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from b601_description.project_config import load_project_config

from b601_target_interfaces.msg import TargetFrame
from b601_target_interfaces.srv import ReleaseSource, RenewSource, SelectSource, SetTarget

from .target_arbiter import ARBITER


class TargetClient:
    def __init__(self, node, source, *, callback_group=None, auto_renew=True):
        self.node = node
        self.source = source
        self.session_id = ""
        self.sequence = 0
        self.lock = Lock()

        project = load_project_config()
        robot = project["robot"]
        self.joints = (*robot["arm_joints"], robot["gripper_joint"])
        self.limits = project["joints"]
        self.feedback_timeout = project["safety"]["feedback_timeout_sec"]
        self.feedback = {}
        self.feedback_at = -float("inf")
        self.positions = {}
        node.create_subscription(
            JointState, "/joint_states", self.on_feedback, qos_profile_sensor_data,
            callback_group=callback_group)

        if auto_renew:
            timeout = project["tools"]["target_arbitration"]["authority_timeout_sec"]
            self.heartbeats = node.create_publisher(String, f"{ARBITER}/source_heartbeat", 10)
            self.heartbeat_timer = node.create_timer(
                timeout / 5, self.heartbeat, callback_group=callback_group,
                clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publisher = node.create_publisher(TargetFrame, f"{ARBITER}/target", 10)
        self.selector = node.create_client(
            SelectSource, f"{ARBITER}/select_source", callback_group=callback_group)
        self.releaser = node.create_client(
            ReleaseSource, f"{ARBITER}/release_source", callback_group=callback_group)
        self.renewer = node.create_client(
            RenewSource, f"{ARBITER}/renew_source", callback_group=callback_group)
        self.sender = node.create_client(
            SetTarget, f"{ARBITER}/set_target", callback_group=callback_group)

    def call(self, client, request, timeout, *, spinning=False):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"target arbitration service unavailable: {client.srv_name}")
        future = client.call_async(request)
        if spinning:
            completed = Event()
            future.add_done_callback(lambda _: completed.set())
            completed.wait(timeout)
        else:
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            client.remove_pending_request(future)
            raise RuntimeError(f"target arbitration service timed out: {client.srv_name}")
        return future.result()

    def heartbeat(self):
        with self.lock:
            session_id = self.session_id
        if session_id:
            self.heartbeats.publish(String(data=session_id))

    def select(self, timeout, *, spinning=False):
        with self.lock:
            if self.session_id:
                raise RuntimeError("release the previous target session before selecting again")
        request = SelectSource.Request(source=self.source)
        reply = self.call(self.selector, request, timeout, spinning=spinning)
        if not reply.accepted:
            raise RuntimeError(reply.message)
        with self.lock:
            self.session_id = reply.session_id
            self.sequence = 0
            self.positions = {}

    def renew(self, timeout, *, spinning=False):
        with self.lock:
            request = RenewSource.Request(session_id=self.session_id)
        reply = self.call(self.renewer, request, timeout, spinning=spinning)
        if not reply.accepted:
            raise RuntimeError(reply.message)

    def release(self, timeout, *, spinning=False):
        with self.lock:
            session_id = self.session_id
        if not session_id:
            return
        request = ReleaseSource.Request(session_id=session_id)
        reply = self.call(self.releaser, request, timeout, spinning=spinning)
        with self.lock:
            self.session_id = ""
        if not reply.accepted:
            raise RuntimeError(reply.message)

    def frame(self, target):
        with self.lock:
            if not self.session_id:
                raise RuntimeError("select the target source before publishing")
            self.sequence += 1
            return TargetFrame(session_id=self.session_id, sequence=self.sequence, target=target)

    def on_feedback(self, message):
        with self.lock:
            self.feedback = dict(zip(message.name, message.position))
            self.feedback_at = monotonic()

    def complete_target(self, message):
        """Expand local joint edits into one full joint target for the ROS interface."""
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        if (not positions or len(positions) != len(message.name)
                or len(message.position) != len(message.name)
                or (message.velocity and len(message.velocity) != len(message.name))
                or message.effort or not positions.keys() <= set(self.joints)):
            raise ValueError("invalid joint target")
        with self.lock:
            values = []
            for name in self.joints:
                bounds = self.limits[name]
                if name in positions:
                    value = positions[name]
                elif name in self.positions:
                    value = self.positions[name]
                else:
                    value = self.feedback.get(name, float("nan"))
                    if monotonic() - self.feedback_at > self.feedback_timeout or not isfinite(value):
                        raise RuntimeError(f"waiting for joint feedback for {name}")
                    value = min(bounds["max_pos"], max(bounds["min_pos"], value))
                values.append(value)
        return JointState(
            header=message.header, name=list(self.joints), position=values,
            velocity=[velocities.get(name, 0.0) for name in self.joints] if velocities else [],
        )

    def remember_target(self, frame):
        target = frame.target
        with self.lock:
            if frame.session_id == self.session_id and frame.sequence == self.sequence:
                self.positions = dict(zip(target.name, target.position))

    def publish(self, message):
        frame = self.frame(self.complete_target(message))
        self.publisher.publish(frame)
        if (all(self.limits[name]["min_pos"] <= value <= self.limits[name]["max_pos"]
                for name, value in zip(frame.target.name, frame.target.position))
                and all(isfinite(value) for value in frame.target.velocity)):
            self.remember_target(frame)

    def set_joints(self, positions, timeout, *, spinning=False):
        message = JointState(name=list(positions), position=list(positions.values()))
        frame = self.frame(self.complete_target(message))
        reply = self.call(self.sender, SetTarget.Request(frame=frame), timeout, spinning=spinning)
        if not reply.accepted:
            raise RuntimeError(reply.message)
        self.remember_target(frame)
        return reply.dispatch_id

    def set_joint(self, joint, position, timeout, *, spinning=False):
        return self.set_joints({joint: position}, timeout, spinning=spinning)

    def close(self, timeout):
        if rclpy.ok():
            try:
                self.release(timeout)
            except RuntimeError as error:
                self.node.get_logger().warning(str(error))
