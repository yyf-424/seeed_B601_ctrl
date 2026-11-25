"""Exclusive MIT joint-target sessions and routing."""
import json
from dataclasses import dataclass, field
from math import isfinite
from time import monotonic_ns
from uuid import uuid4

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, UInt64

from b601_target_interfaces.msg import AuthorizedTarget, TargetFrame
from b601_target_interfaces.srv import ReleaseSource, RenewSource, RevokeSource, SelectSource, SetTarget

from b601_description.project_config import load_project_config

ARBITER = "/b601_target_arbiter"


@dataclass
class Session:
    source: str
    controllers: tuple
    expires_at: int
    status_id: str
    sequence: int = 0
    positions: dict = field(default_factory=dict)


class TargetArbiter(Node):
    def __init__(self):
        super().__init__("b601_target_arbiter")
        project_path = self.declare_parameter("project_config_file", "").value
        project = load_project_config(project_path or None)
        routes = self.declare_parameter("routes", "{}").value
        self.routes = {name: tuple(joints) for name, joints in json.loads(routes).items()}
        self.limits = project["joints"]
        if not self.routes:
            raise ValueError("No MIT target controller routes")
        if any(not names or any(j not in self.limits for j in names)
               for names in self.routes.values()):
            raise ValueError("Invalid MIT joint route")
        self.feedback = {}
        self.sessions = {}
        self.owners = {}
        self.dispatched = {}
        self.applied = {}
        self.feedback_stamp = None
        self.feedback_ready = dict.fromkeys(self.routes, False)
        self.feedback_timeout_ns = project["safety"]["feedback_timeout_sec"] * 1e9
        authority_timeout = project["tools"]["target_arbitration"]["authority_timeout_sec"]
        self.authority_timeout_ns = int(authority_timeout * 1e9)
        self.holding = {}
        self.epochs = {name: self.new_epoch() for name in self.routes}
        self.last_heartbeat_ns = monotonic_ns()
        self.dispatch_id = monotonic_ns()
        self.hold_revisions = dict.fromkeys(self.routes, 0)
        self.outputs = {}
        for controller in self.routes:
            self.outputs[controller] = self.create_publisher(
                AuthorizedTarget, f"/{controller}/authorized_target", 10)
            self.create_subscription(
                UInt64, f"/{controller}/target_applied",
                lambda msg, controller=controller: self.on_applied(controller, msg), 10,
            )
            self.create_subscription(
                Bool, f"/{controller}/target_holding",
                lambda msg, controller=controller: self.on_holding(controller, msg), 10,
            )
            self.create_subscription(
                UInt64, f"/{controller}/target_epoch_requested",
                lambda msg, controller=controller: self.on_epoch_requested(controller, msg), 10,
            )
        self.status = self.create_publisher(
            String, "~/status", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, "/joint_states", self.on_feedback, qos_profile_sensor_data)
        self.create_subscription(TargetFrame, "~/target", self.on_target, 10)
        self.create_subscription(String, "~/source_heartbeat", self.source_heartbeat, 10)
        self.create_service(SelectSource, "~/select_source", self.select_source)
        self.create_service(ReleaseSource, "~/release_source", self.release_source)
        self.create_service(RevokeSource, "~/revoke_source", self.revoke_source)
        self.create_service(RenewSource, "~/renew_source", self.renew_source)
        self.create_timer(
            authority_timeout / 5, self.heartbeat,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_service(SetTarget, "~/set_target", self.set_target)
        self.last_event = {"event": "ready"}
        self.publish_status()

    def on_feedback(self, message):
        self.feedback = {key: value for key, value in zip(message.name, message.position)
                         if isfinite(value)}
        self.feedback_stamp = monotonic_ns()

    def select_source(self, request, response):
        now = monotonic_ns()
        self.expire_sessions(now)
        source = request.source.strip()
        controllers = tuple(dict.fromkeys(request.controllers))
        if not source or not controllers:
            response.message = "source and controllers are required"
        elif unknown := set(controllers).difference(self.routes):
            response.message = f"unknown controllers: {', '.join(sorted(unknown))}"
        elif busy := [name for name in controllers if name in self.owners]:
            response.message = "control held: " + ", ".join(
                f"{name} by {self.sessions[self.owners[name]].source}" for name in busy)
        else:
            session_id = uuid4().hex
            public_id = uuid4().hex
            self.sessions[session_id] = Session(
                source, controllers, now + self.authority_timeout_ns, public_id)
            for name in controllers:
                self.owners[name] = session_id
                self.publish_heartbeat(name)
            response.accepted = True
            response.session_id = session_id
            response.session = public_id
            response.message = "granted"
        self.event("select", source, response.message)
        return response

    @staticmethod
    def new_epoch():
        return int(uuid4().hex[:16], 16) or 1

    def publish_heartbeat(self, controller):
        self.outputs[controller].publish(AuthorizedTarget(
            kind=AuthorizedTarget.HEARTBEAT, arbiter_id=self.epochs[controller],
            hold_revision=self.hold_revisions[controller]))

    def request_hold(self, controller):
        self.hold_revisions[controller] += 1
        self.publish_heartbeat(controller)

    def heartbeat(self):
        now = monotonic_ns()
        self.expire_sessions(now)
        self.announce_heartbeat(now)
        available = {
            controller: self.feedback_is_fresh(now) and all(name in self.feedback for name in joints)
            for controller, joints in self.routes.items()
        }
        if available != self.feedback_ready:
            self.feedback_ready = available
            self.publish_status()

    def feedback_is_fresh(self, now):
        return self.feedback_stamp is not None and now - self.feedback_stamp <= self.feedback_timeout_ns

    def announce_heartbeat(self, now):
        self.last_heartbeat_ns = now
        for controller in self.outputs:
            self.publish_heartbeat(controller)

    def expire_sessions(self, now):
        if now - self.last_heartbeat_ns >= self.authority_timeout_ns:
            for session_id in list(self.sessions):
                self.close_session(session_id)
            self.epochs = {name: self.new_epoch() for name in self.routes}
            self.announce_heartbeat(now)
            self.event("arbiter_timeout", "", "all prior sessions revoked after arbiter interruption")
            return
        expired = [key for key, session in self.sessions.items() if session.expires_at <= now]
        for session_id in expired:
            session = self.close_session(session_id)
            for controller in session.controllers:
                self.request_hold(controller)
            self.event("expire", session.source, "source heartbeat timed out; hold requested")

    def close_session(self, session_id):
        session = self.sessions.pop(session_id, None)
        if session is not None:
            for controller in session.controllers:
                del self.owners[controller]
        return session

    def release_source(self, request, response):
        self.expire_sessions(monotonic_ns())
        session = self.close_session(request.session_id)
        response.accepted = session is not None
        response.message = "released" if session else "unknown or released session"
        self.event("release", session.source if session else "", response.message)
        return response

    def revoke_source(self, request, response):
        self.expire_sessions(monotonic_ns())
        session_id = self.owners.get(request.controller)
        session = self.sessions.get(session_id)
        if session is None or session.status_id != request.session:
            response.message = "controller no longer has that session"
        else:
            self.close_session(session_id)
            response.accepted = True
            response.message = "revoked"
        self.event("revoke", session.source if session else "", response.message)
        return response

    def keep_alive(self, session_id):
        now = monotonic_ns()
        self.expire_sessions(now)
        session = self.sessions.get(session_id)
        if session is None:
            return False
        session.expires_at = now + self.authority_timeout_ns
        return True

    def source_heartbeat(self, message):
        self.keep_alive(message.data)

    def renew_source(self, request, response):
        response.accepted = self.keep_alive(request.session_id)
        response.message = "renewed" if response.accepted else "unknown or expired session"
        return response

    def on_target(self, frame):
        self.dispatch(frame)

    def set_target(self, request, response):
        response.accepted, response.message, response.dispatch_id = self.dispatch(request.frame)
        return response

    def dispatch(self, frame):
        now = monotonic_ns()
        self.expire_sessions(now)
        session = self.sessions.get(frame.session_id)
        source = session.source if session else ""
        try:
            commands = self.prepare(frame, session)
        except ValueError as error:
            self.event("reject", source, str(error))
            return False, str(error), 0

        self.dispatch_id += 1
        session.sequence = frame.sequence
        session.expires_at = now + self.authority_timeout_ns
        for controller, target in commands.items():
            session.positions.update(zip(target.name, target.position))
            self.dispatched[controller] = {
                "source": source, "session": session.status_id,
                "sequence": frame.sequence, "dispatch_id": str(self.dispatch_id),
                "position": dict(zip(target.name, target.position)),
            }
            self.outputs[controller].publish(
                AuthorizedTarget(kind=AuthorizedTarget.TARGET, arbiter_id=self.epochs[controller],
                                 hold_revision=self.hold_revisions[controller],
                                 dispatch_id=self.dispatch_id, target=target))
        self.event("dispatch", source, "accepted")
        return True, "accepted for dispatch", self.dispatch_id

    def prepare(self, frame, session):
        if session is None:
            raise ValueError("unknown or released session")
        if frame.sequence <= session.sequence:
            raise ValueError("sequence must increase")
        msg = frame.target
        names = msg.name
        if (not names or len(set(names)) != len(names) or len(names) != len(msg.position)
                or (msg.velocity and len(msg.velocity) != len(names)) or msg.effort):
            raise ValueError("send distinct joint names, positions and optional velocities")
        controlled = {key for controller in session.controllers for key in self.routes[controller]}
        if unknown := set(names).difference(controlled):
            raise ValueError(f"session does not control: {', '.join(sorted(unknown))}")
        velocities = dict(zip(names, msg.velocity))
        positions = dict(zip(names, msg.position))
        if not all(isfinite(value) for value in [*positions.values(), *velocities.values()]):
            raise ValueError("non-finite target")
        for name, value in positions.items():
            bounds = self.limits[name]
            if not bounds["min_pos"] <= value <= bounds["max_pos"]:
                raise ValueError(f"{name} outside configured position bounds")
        commands = {}
        for controller in session.controllers:
            joints = self.routes[controller]
            if set(joints).isdisjoint(positions):
                continue
            current = self.feedback if self.feedback_is_fresh(monotonic_ns()) else {}
            filled = {
                name: positions.get(name, session.positions.get(name, current.get(name)))
                for name in joints
            }
            if any(value is None for value in filled.values()):
                raise ValueError(f"waiting for joint feedback for {controller}")
            commands[controller] = JointState(
                name=list(joints), position=list(filled.values()),
                velocity=[velocities.get(name, 0.0) for name in joints] if velocities else [],
            )
        return commands

    def on_applied(self, controller, message):
        previous = self.applied.get(controller, 0)
        if previous == message.data:
            return
        if message.data:
            self.applied[controller] = message.data
        else:
            self.applied.pop(controller, None)
        self.publish_status()

    def on_epoch_requested(self, controller, message):
        if message.data != self.epochs[controller]:
            return
        session = self.close_session(self.owners.get(controller))
        self.epochs[controller] = self.new_epoch()
        self.publish_heartbeat(controller)
        if session:
            for name in session.controllers:
                if name != controller:
                    self.request_hold(name)
        self.event("controller_timeout", session.source if session else "",
                   f"{controller} target bridge lost arbiter heartbeat; previous authority revoked")

    def on_holding(self, controller, message):
        if self.holding.get(controller) != message.data:
            self.holding[controller] = message.data
            self.publish_status()

    def event(self, event, source, message):
        self.last_event = {"event": event, "source": source, "message": message}
        self.publish_status()

    def publish_status(self):
        state = {}
        for controller, joints in self.routes.items():
            session_id = self.owners.get(controller)
            session = self.sessions.get(session_id)
            state[controller] = {
                "joints": joints,
                "feedback_ready": self.feedback_ready[controller],
                "source": session.source if session else None,
                "session": session.status_id if session else None,
                "holding_after_loss": self.holding.get(controller, False),
                "last_forwarded": self.dispatched.get(controller),
                "last_applied_id": str(self.applied[controller]) if controller in self.applied else None,
            }
        self.status.publish(String(data=json.dumps(
            {"controllers": state, "last_event": self.last_event}, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = TargetArbiter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
