"""Local HTTP and ROS bridge for the B601 target authorization console."""
import json
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Event, Thread
from urllib.parse import urlsplit
from xml.etree.ElementTree import ParseError

import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import DynamicJointState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from b601_description.project_config import load_project_config
from b601_target_interfaces.msg import TargetFrame
from b601_target_interfaces.srv import ReleaseSource, RenewSource, RevokeSource, SelectSource, SetAuthority, SetTarget

from .mit_recenter import RECENTER_SERVICE
from .mit_recenter_client import recenter_result_timeout
from .target_arbiter import ARBITER
from .web_backend import WebBackend
from .web_robot_model import build_model, find_vendor_model


class TargetWeb(Node):
    def __init__(self):
        super().__init__("b601_target_web")
        self.port = self.declare_parameter("port", 8765).value
        config_path = self.declare_parameter("project_config_file", "").value
        config = load_project_config(config_path or None)
        attached = self.declare_parameter("attached_backend", "").value
        attached_scene = self.declare_parameter("attached_mujoco_scene", "").value
        self.backend = WebBackend(self, config, config_path, attached, attached_scene)
        self.feedback_timeout = config["safety"]["feedback_timeout_sec"]
        self.heartbeat_period_ms = round(
            config["tools"]["target_arbitration"]["authority_timeout_sec"] * 200)
        self.limits = {joint: {"min": bounds["min_pos"], "max": bounds["max_pos"]}
                       for joint, bounds in config["joints"].items()}
        self.status = None
        self.feedback = {}
        self.feedback_at = None
        self.stiffness_sample = None
        self.robot_model = None
        self.vendor_model = find_vendor_model()
        self.create_subscription(
            String, "/robot_description", self.on_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            String, f"{ARBITER}/status", self.on_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, "/joint_states", self.on_feedback, qos_profile_sensor_data)
        self.create_subscription(
            DynamicJointState, "/dynamic_joint_states", self.on_stiffness_feedback, qos_profile_sensor_data)
        self.select = self.create_client(SelectSource, f"{ARBITER}/select_source")
        self.authority = self.create_client(SetAuthority, f"{ARBITER}/set_authority")
        self.renew = self.create_client(RenewSource, f"{ARBITER}/renew_source")
        self.release = self.create_client(ReleaseSource, f"{ARBITER}/release_source")
        self.revoke = self.create_client(RevokeSource, f"{ARBITER}/revoke_source")
        self.set_target = self.create_client(SetTarget, f"{ARBITER}/set_target")
        self.recenter = self.create_client(Trigger, RECENTER_SERVICE)
        self.release_for_recenter = self.create_client(ReleaseSource, f"{ARBITER}/release_web_for_recenter")
        self.recenter_timeout = recenter_result_timeout(config)

    def on_status(self, message):
        self.status = json.loads(message.data)

    def on_feedback(self, message):
        self.feedback = dict(zip(message.name, message.position))
        self.feedback_at = time.monotonic()

    def on_stiffness_feedback(self, message):
        joint = next((values for name, values in zip(message.joint_names, message.interface_values)
                      if name == "joint1"), None)
        flag = dict(zip(joint.interface_names, joint.values)).get("low_stiffness") if joint else None
        self.stiffness_sample = (flag, time.monotonic()) if flag in (0.0, 1.0) else None

    def on_description(self, message):
        try:
            self.robot_model = build_model(message.data, self.vendor_model)
        except (KeyError, ValueError, ParseError) as error:
            self.get_logger().warning(f"网页机械臂模型读取失败：{error}")

    def snapshot(self):
        feedback_at = self.feedback_at
        now = time.monotonic()
        age = now - feedback_at if feedback_at is not None else None
        status = self.status if self.select.service_is_ready() else None
        fresh = age is not None and age <= self.feedback_timeout
        backend = self.backend.snapshot(
            fresh and bool(status and status["feedback_ready"]),
            fresh and bool(status and status.get("disable_ready")))
        sample = self.stiffness_sample
        stiffness = None
        if (backend["phase"] == "connected" and self.backend.active and sample is not None
                and now - sample[1] <= self.feedback_timeout):
            stiffness = "low" if sample[0] == 1.0 else "high"
        return {
            "status": status,
            "backend": backend,
            "stiffness": stiffness,
            "recenter_ready": self.recenter.service_is_ready(),
            "feedback": self.feedback,
            "limits": self.limits,
            "heartbeat_period_ms": self.heartbeat_period_ms,
            "feedback_age_ms": round(age * 1000) if age is not None else None,
        }

    def request(self, path, data):
        if path == "/api/connect":
            return self.backend.connect(data["backend"], data.get("mujoco_scene", "free"))
        if path == "/api/disable":
            return self.backend.disable(self.call_service)
        if path == "/api/recenter":
            return self.request_recenter(data.get("session_id"))
        if path == "/api/select":
            request = SelectSource.Request(source="web")
            client = self.select
        elif path == "/api/authority":
            request = SetAuthority.Request(source=data["source"])
            client = self.authority
        elif path == "/api/heartbeat":
            request = RenewSource.Request(session_id=data["session_id"])
            client = self.renew
        elif path == "/api/release":
            request = ReleaseSource.Request(session_id=data["session_id"])
            client = self.release
        elif path == "/api/revoke":
            request = RevokeSource.Request(session=data["session"])
            client = self.revoke
        elif path == "/api/target":
            positions = data["positions"]
            if not isinstance(positions, dict):
                raise ValueError("positions must be an object keyed by joint name")
            target = JointState(name=list(positions), position=[float(v) for v in positions.values()])
            request = SetTarget.Request(frame=TargetFrame(
                session_id=data["session_id"], sequence=data["sequence"], target=target))
            client = self.set_target
        else:
            raise KeyError("unknown endpoint")

        status, reply = self.call_service(client, request)
        if status != HTTPStatus.OK:
            return status, reply
        result = {"accepted": reply.accepted, "message": reply.message}
        if isinstance(reply, SelectSource.Response) and reply.accepted:
            result["session_id"] = reply.session_id
            result["session"] = reply.session
        if isinstance(reply, SetTarget.Response) and reply.accepted:
            result["dispatch_id"] = str(reply.dispatch_id)
        return HTTPStatus.OK if reply.accepted else HTTPStatus.CONFLICT, result

    def request_recenter(self, session_id):
        if not self.recenter.service_is_ready():
            return HTTPStatus.SERVICE_UNAVAILABLE, {"message": "归中服务未就绪"}
        if session_id:
            request = ReleaseSource.Request(session_id=session_id)
            status, reply = self.call_service(self.release_for_recenter, request)
            if status != HTTPStatus.OK:
                return status, reply
            if not reply.accepted:
                return HTTPStatus.CONFLICT, {"message": reply.message}
        status, reply = self.call_service(self.recenter, Trigger.Request(), self.recenter_timeout)
        if status == HTTPStatus.GATEWAY_TIMEOUT:
            return status, {"message": "未收到归中结果，请查看当前机器人状态"}
        if status != HTTPStatus.OK:
            return status, reply
        return (HTTPStatus.OK if reply.success else HTTPStatus.CONFLICT,
                {"accepted": reply.success, "message": reply.message})

    @staticmethod
    def call_service(client, request, timeout=3):
        if not client.service_is_ready():
            return HTTPStatus.SERVICE_UNAVAILABLE, {"message": "ROS 服务未就绪"}
        future = client.call_async(request)
        complete = Event()
        future.add_done_callback(lambda _: complete.set())
        if not complete.wait(timeout):
            client.remove_pending_request(future)
            return HTTPStatus.GATEWAY_TIMEOUT, {"message": "ROS 服务响应超时"}
        return HTTPStatus.OK, future.result()


class TargetHandler(BaseHTTPRequestHandler):
    asset_files = {
        "/": ("index.html", "text/html"),
        "/app.js": ("app.js", "text/javascript"),
        "/style.css": ("style.css", "text/css"),
        "/robot-view.js": ("robot-view.js", "text/javascript"),
        "/tcp-ik.js": ("tcp-ik.js", "text/javascript"),
        "/vendor/three.module.min.js": ("vendor/three.module.min.js", "text/javascript"),
        "/vendor/OrbitControls.js": ("vendor/OrbitControls.js", "text/javascript"),
        "/vendor/STLLoader.js": ("vendor/STLLoader.js", "text/javascript"),
        "/vendor/THREE-LICENSE.txt": ("vendor/THREE-LICENSE.txt", "text/plain"),
    }

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/api/state":
            self.send_json(HTTPStatus.OK, self.server.bridge.snapshot())
            return
        robot = self.server.bridge.robot_model
        if path == "/api/model":
            if robot is None:
                self.send_json(HTTPStatus.SERVICE_UNAVAILABLE, {"message": "等待机械臂模型"})
            else:
                self.send_json(HTTPStatus.OK, robot[0])
            return
        if robot is not None and (mesh := robot[1].get(path)) is not None:
            try:
                data = mesh.read_bytes()
            except OSError:
                self.send_error(HTTPStatus.NOT_FOUND)
            else:
                self.send_content(HTTPStatus.OK, "model/stl", data, cache="public, max-age=3600")
            return
        asset = self.asset_files.get(path)
        if asset is None:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        filename, content_type = asset
        self.send_content(HTTPStatus.OK, content_type, (self.server.assets / filename).read_bytes())

    def do_POST(self):
        if self.headers.get("Host") != f"127.0.0.1:{self.server.server_port}":
            self.send_json(HTTPStatus.FORBIDDEN, {"message": "使用页面的本地地址"})
            return
        origin = self.headers.get("Origin")
        if origin and origin != f"http://127.0.0.1:{self.server.server_port}":
            self.send_json(HTTPStatus.FORBIDDEN, {"message": "请求来源与页面不一致"})
            return
        if self.headers.get_content_type() != "application/json":
            self.send_json(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, {"message": "需要 JSON"})
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            if not 0 < length <= 65536:
                raise ValueError("invalid request size")
            data = json.loads(self.rfile.read(length))
            code, body = self.server.bridge.request(urlsplit(self.path).path, data)
        except (ValueError, TypeError, KeyError, AssertionError, OverflowError) as error:
            code, body = HTTPStatus.BAD_REQUEST, {"message": str(error)}
        self.send_json(code, body)

    def log_message(self, _format, *_args):
        pass

    def send_json(self, status, data):
        self.send_content(status, "application/json", json.dumps(data, ensure_ascii=False).encode())

    def send_content(self, status, content_type, data, *, cache="no-store"):
        self.send_response(status)
        if content_type.startswith("text/") or content_type == "application/json":
            content_type += "; charset=utf-8"
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", cache)
        self.end_headers()
        self.wfile.write(data)


def main(args=None):
    rclpy.init(args=args)
    node = TargetWeb()
    with ThreadingHTTPServer(("127.0.0.1", node.port), TargetHandler) as server:
        server.bridge = node
        server.assets = Path(get_package_share_directory("b601_demos")) / "web"
        thread = Thread(target=server.serve_forever, daemon=True)
        thread.start()
        node.get_logger().info(f"Target console: http://127.0.0.1:{server.server_port}")
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except RuntimeError:
            if rclpy.ok():
                raise
        finally:
            server.shutdown()
            thread.join()
            node.destroy_node()
            rclpy.try_shutdown()
