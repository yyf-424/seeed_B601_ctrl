"""Client for selecting a source and supplying sequenced MIT joint targets."""
from threading import Event, Lock

import rclpy
from rclpy.clock import Clock, ClockType
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

        if auto_renew:
            timeout = load_project_config()["tools"]["target_arbitration"]["authority_timeout_sec"]
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

    def select(self, controllers, timeout, *, spinning=False):
        with self.lock:
            if self.session_id:
                raise RuntimeError("release the previous target session before selecting again")
        request = SelectSource.Request(source=self.source, controllers=list(controllers))
        reply = self.call(self.selector, request, timeout, spinning=spinning)
        if not reply.accepted:
            raise RuntimeError(reply.message)
        with self.lock:
            self.session_id = reply.session_id
            self.sequence = 0

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

    def publish(self, message):
        self.publisher.publish(self.frame(message))

    def set_joints(self, positions, timeout, *, spinning=False):
        message = JointState(name=list(positions), position=list(positions.values()))
        request = SetTarget.Request(frame=self.frame(message))
        reply = self.call(self.sender, request, timeout, spinning=spinning)
        if not reply.accepted:
            raise RuntimeError(reply.message)
        return reply.dispatch_id

    def set_joint(self, joint, position, timeout, *, spinning=False):
        return self.set_joints({joint: position}, timeout, spinning=spinning)

    def close(self, timeout):
        if rclpy.ok():
            try:
                self.release(timeout)
            except RuntimeError as error:
                self.node.get_logger().warning(str(error))
