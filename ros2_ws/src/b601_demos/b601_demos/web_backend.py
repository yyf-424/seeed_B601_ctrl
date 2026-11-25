"""Launch ROS bringup from the standalone local console and report its lifecycle."""
import os
import signal
import subprocess
from contextlib import ExitStack
from fcntl import LOCK_EX, LOCK_NB, flock
from http import HTTPStatus
from pathlib import Path
from tempfile import NamedTemporaryFile
from threading import Lock
from time import monotonic

from controller_manager_msgs.srv import ListControllers


BACKEND_NAMES = {"mock": "Mock", "mujoco": "MuJoCo", "dm": "实机"}


def bringup_command(backend, project_file):
    args = [
        "ros2", "launch", "b601_bringup", "bringup.launch.py",
        f"hardware:={backend}", "dm_command_mode:=mit_impedance",
        "use_rviz:=false", "use_safety:=true", "use_target_web:=false",
        "mit_recenter_on_startup:=false",
    ]
    if backend == "dm":
        args.extend(("dm_enable_motors:=true", "mit_gravity_compensation:=true"))
    if project_file:
        args.append(f"project_config_file:={project_file}")
    return args


class WebBackend:
    def __init__(self, node, project, project_file, attached):
        self.node = node
        self.project_file = project_file
        self.attached = attached
        self.device = Path(project["hardware"]["dm"]["device"])
        self.startup_timeout = project["tools"]["sim_mit_ros"]["startup_timeout"]
        self.controllers = {"joint_state_broadcaster", *project["controllers"]["mit"]["instances"]}
        self.controller_list = node.create_client(ListControllers, "/controller_manager/list_controllers")
        self.controller_request = None
        self.active = False
        self.lock = Lock()
        self.process = None
        self.backend = None
        self.started_at = None
        self.stopping = False
        self.error = None
        self.log_path = None
        node.create_timer(0.5, self.update_controllers)

    def update_controllers(self):
        if not self.controller_list.service_is_ready():
            self.active = False
            if self.controller_request is not None:
                self.controller_list.remove_pending_request(self.controller_request)
                self.controller_request = None
            return
        if self.controller_request is None:
            self.controller_request = self.controller_list.call_async(ListControllers.Request())
        elif self.controller_request.done():
            result = self.controller_request.result()
            self.controller_request = None
            active = {c.name for c in result.controller if c.state == "active"} if result else set()
            self.active = self.controllers <= active

    def graph_in_use(self):
        return (self.controller_list.service_is_ready()
                or bool(self.node.count_publishers("/joint_states"))
                or "controller_manager" in self.node.get_node_names())

    def reap(self):
        if self.process is not None and (result := self.process.poll()) is not None:
            self.error = None if self.stopping else f"{BACKEND_NAMES[self.backend]} 后端已退出（{result}）"
            self.backend = None
            self.process = None
            self.stopping = False
            self.active = False

    def snapshot(self, feedback_ready):
        with self.lock:
            self.reap()
            active_backend = self.attached or self.backend
            attached = bool(self.attached) or (self.process is None and self.graph_in_use())
            if self.stopping:
                phase, message = "stopping", "正在断开 MuJoCo…"
            elif self.backend:
                label = BACKEND_NAMES[self.backend]
                if self.active and self.controller_list.service_is_ready() and feedback_ready:
                    detail = "电机已使能" if self.backend == "dm" else "MIT 控制器已激活"
                    phase, message = "connected", f"{label} 已连接 · {detail}"
                elif monotonic() - self.started_at >= self.startup_timeout:
                    phase, message = "error", f"{label} 控制器或反馈未就绪，请查看启动日志"
                else:
                    phase, message = "starting", f"正在连接 {label}…"
            elif attached:
                phase = "connected" if feedback_ready else "starting"
                label = BACKEND_NAMES.get(self.attached, "现有后端")
                message = f"{label} 已连接" if feedback_ready else f"等待 {label} 反馈…"
            elif self.error:
                phase, message = "error", self.error
            else:
                phase, message = "idle", "等待连接"
            return {
                "name": active_backend,
                "phase": phase,
                "message": message,
                "can_connect": not attached and self.process is None,
                "can_disconnect": self.backend == "mujoco" and not self.stopping,
                "log": str(self.log_path) if phase == "error" and self.log_path else None,
            }

    def connect(self, backend):
        if not isinstance(backend, str) or backend not in BACKEND_NAMES:
            return HTTPStatus.BAD_REQUEST, {"message": "请选择 Mock、MuJoCo 或实机"}
        if backend == "mock":
            return HTTPStatus.OK, {"message": "Mock 已选择，未启动后端"}
        with self.lock:
            self.reap()
            if self.process is not None or self.attached or self.graph_in_use():
                return HTTPStatus.CONFLICT, {"message": "当前 ROS 域已有后端运行"}
            try:
                self.process = self.launch(backend)
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
                return HTTPStatus.SERVICE_UNAVAILABLE, {"message": str(error)}
            self.backend = backend
            self.started_at = monotonic()
            self.stopping = False
            self.error = None
            self.active = False
            self.node.get_logger().info(
                f"{BACKEND_NAMES[backend]} bringup PID {self.process.pid}，日志 {self.log_path}")
        return HTTPStatus.ACCEPTED, {"message": f"正在连接 {BACKEND_NAMES[backend]}…"}

    def launch(self, backend):
        directory = Path(os.environ.get("ROS_LOG_DIR") or Path.home() / ".ros" / "log")
        directory.mkdir(parents=True, exist_ok=True)
        locks = Path(os.environ.get("XDG_RUNTIME_DIR") or Path.home() / ".ros")
        locks.mkdir(parents=True, exist_ok=True)
        domain = self.node.context.get_domain_id()
        names = [f"b601-web-domain-{domain}"]
        if backend == "dm":
            if not self.device.is_char_device():
                raise RuntimeError(f"找不到真机串口 {self.device}")
            names.append(f"b601-web-serial-{self.device.stat().st_rdev}")
        with ExitStack() as stack:
            descriptors = []
            for name in names:
                descriptor = stack.enter_context((locks / f"{name}.lock").open("a")).fileno()
                try:
                    flock(descriptor, LOCK_EX | LOCK_NB)
                except BlockingIOError as error:
                    raise RuntimeError("已有网页启动的后端占用当前 ROS 域或串口") from error
                descriptors.append(descriptor)
            if backend == "dm":
                scan = subprocess.run(["fuser", str(self.device)], capture_output=True, timeout=3)
                if scan.returncode != 1:
                    raise RuntimeError(f"未确认真机串口空闲 {self.device}")
            output = stack.enter_context(NamedTemporaryFile(
                prefix=f"b601-web-{backend}-", suffix=".log", dir=directory, delete=False))
            self.log_path = Path(output.name)
            return subprocess.Popen(
                bringup_command(backend, self.project_file), start_new_session=True,
                env={**os.environ, "ROS_DOMAIN_ID": str(domain)},
                stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                pass_fds=tuple(descriptors))

    def disconnect_simulation(self):
        with self.lock:
            self.reap()
            if self.backend != "mujoco" or self.stopping:
                return HTTPStatus.CONFLICT, {"message": "当前没有由网页启动的 MuJoCo 可断开"}
            self.process.send_signal(signal.SIGINT)
            self.stopping = True
        return HTTPStatus.ACCEPTED, {"message": "正在断开 MuJoCo…"}
