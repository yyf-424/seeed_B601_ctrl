"""Local HTTP and ROS bridge for the B601 target authorization console."""
import json
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Event, Thread
from urllib.parse import urlsplit

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from b601_description.project_config import load_project_config
from b601_target_interfaces.msg import TargetFrame
from b601_target_interfaces.srv import ReleaseSource, RenewSource, SelectSource, SetTarget

from .target_arbiter import ARBITER


class TargetWeb(Node):
    def __init__(self):
        super().__init__("b601_target_web")
        self.port = self.declare_parameter("port", 8765).value
        config_path = self.declare_parameter("project_config_file", "").value
        config = load_project_config(config_path or None)
        self.heartbeat_period_ms = round(
            config["implementation"]["mit"]["target_authority_timeout_sec"] * 200)
        self.limits = {joint: {"min": bounds["min_pos"], "max": bounds["max_pos"]}
                       for joint, bounds in config["joints"].items()}
        self.status = None
        self.feedback = {}
        self.feedback_at = None
        self.create_subscription(
            String, f"{ARBITER}/status", self.on_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, "/joint_states", self.on_feedback, qos_profile_sensor_data)
        self.select = self.create_client(SelectSource, f"{ARBITER}/select_source")
        self.renew = self.create_client(RenewSource, f"{ARBITER}/renew_source")
        self.release = self.create_client(ReleaseSource, f"{ARBITER}/release_source")
        self.set_target = self.create_client(SetTarget, f"{ARBITER}/set_target")

    def on_status(self, message):
        self.status = json.loads(message.data)

    def on_feedback(self, message):
        self.feedback = dict(zip(message.name, message.position))
        self.feedback_at = time.monotonic()

    def snapshot(self):
        feedback_at = self.feedback_at
        return {
            "status": self.status if self.select.service_is_ready() else None,
            "feedback": self.feedback,
            "limits": self.limits,
            "heartbeat_period_ms": self.heartbeat_period_ms,
            "feedback_age_ms": round((time.monotonic() - feedback_at) * 1000) if feedback_at else None,
        }

    def request(self, path, data):
        if path == "/api/select":
            request = SelectSource.Request(source="web", controllers=[data["controller"]])
            client = self.select
        elif path == "/api/heartbeat":
            request = RenewSource.Request(session_id=data["session_id"])
            client = self.renew
        elif path == "/api/release":
            request = ReleaseSource.Request(session_id=data["session_id"])
            client = self.release
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

        if not client.service_is_ready():
            return HTTPStatus.SERVICE_UNAVAILABLE, {"message": "目标仲裁服务未就绪"}
        future = client.call_async(request)
        complete = Event()
        future.add_done_callback(lambda _: complete.set())
        if not complete.wait(3):
            client.remove_pending_request(future)
            return HTTPStatus.GATEWAY_TIMEOUT, {"message": "目标仲裁服务响应超时"}
        reply = future.result()
        result = {"accepted": reply.accepted, "message": reply.message}
        if isinstance(reply, SelectSource.Response) and reply.accepted:
            result["session_id"] = reply.session_id
            result["session"] = reply.session
        if isinstance(reply, SetTarget.Response) and reply.accepted:
            result["dispatch_id"] = str(reply.dispatch_id)
        return HTTPStatus.OK if reply.accepted else HTTPStatus.CONFLICT, result


class TargetHandler(BaseHTTPRequestHandler):
    asset_files = {"/": ("index.html", "text/html"), "/app.js": ("app.js", "text/javascript"),
                   "/style.css": ("style.css", "text/css")}

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/api/state":
            self.send_json(HTTPStatus.OK, self.server.bridge.snapshot())
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

    def send_content(self, status, content_type, data):
        self.send_response(status)
        self.send_header("Content-Type", f"{content_type}; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
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
