"""safety_monitor.py —— 节点 b601_safety_monitor

角色：**独立的安全监控节点**。只读 /joint_states，判断机械臂当前处于
      正常（NORMAL）/ 警告（WARNING）/ 急停（EMERGENCY_STOP），并把状态发布到一个
      独立话题上；只有在「角度越限」或「角速度越限」时才动手停机。

接口：
    订阅    /joint_states                     sensor_msgs/JointState
    发布    /b601_safety/status               diagnostic_msgs/DiagnosticStatus
                                              （transient_local：后来的订阅者也能立刻拿到当前状态）
    服务    /b601_safety/clear_estop          std_srvs/Trigger（解除锁存，需人工调用）
    服务客户端
            /controller_manager/list_controllers            controller_manager_msgs/ListControllers
            /controller_manager/switch_controller           controller_manager_msgs/SwitchController
            /controller_manager/set_hardware_component_state controller_manager_msgs/SetHardwareComponentState

急停动作（顺序执行，任一步失败只记录日志、继续下一步）：
    ① list_controllers 拿到当前 active 控制器（配置里 deactivate_controllers=["*"] 时全停）
    ② switch_controller 把它们全部 deactivate  → 不再有人写命令
    ③ set_hardware_component_state 把硬件设为 INACTIVE
       → DmSystemHardware::on_deactivate() → motor_controller_disable_all()
       → 这就是"全臂失能"

为什么急停不直接把某个 controller 设成 inactive 就完事：
    控制器停下来只是"不再写新命令"，硬件还会按最后一条命令保持/继续跑。
    真正让电机不再出力，必须让硬件组件进入 INACTIVE（ros2_control 生命周期语义）。

为什么解除急停不自动恢复：
    自动恢复意味着"检测到安全就自己上电"，这在人还在机械臂旁边时最危险。
    必须在确认现场安全后，人工重新起控制器。

参数：见 b601_safety/config/safety_params.yaml（由 scripts/sync_config.py 从
      b601_project.yaml 生成，不要手改）。
"""

from __future__ import annotations

import time

import rclpy
from controller_manager_msgs.srv import (
    ListControllers,
    SetHardwareComponentState,
    SwitchController,
)
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from lifecycle_msgs.msg import State as LifecycleState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from b601_safety.logic import (
    EMERGENCY_STOP,
    LEVEL_ERROR,
    NORMAL,
    WARNING,
    Evaluation,
    SafetyLimits,
    evaluate,
)

ALL_CONTROLLERS = "*"


def _set_diag_level(msg: DiagnosticStatus, level: int) -> None:
    """给 DiagnosticStatus.level 赋值。

    坑：Humble 的 diagnostic_msgs 里 `byte level` 在 Python 侧被生成为
    "长度 1 的 bytes"，直接赋 int 会抛
    AssertionError: The 'level' field must be of type 'bytes' ...
    这里两种都兼容，换 ROS 版本也不会再踩一次。
    """
    try:
        msg.level = level
    except (AssertionError, TypeError):
        msg.level = bytes([level])


def _as_float_list(values) -> list[float]:
    return [float(v) for v in values]


class SafetyMonitor(Node):
    def __init__(self) -> None:
        super().__init__("b601_safety_monitor")

        # 默认参数随包安装，启动时可用标准 ROS 参数覆盖。
        from pathlib import Path

        from ament_index_python.packages import get_package_share_directory
        import yaml

        path = Path(get_package_share_directory("b601_safety")) / "config" / "safety_params.yaml"
        all_params = yaml.safe_load(path.read_text(encoding="utf-8"))
        defaults = next(iter(all_params.values()))["ros__parameters"]
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        names = list(self.get_parameter("joint_names").value)
        pos_min = _as_float_list(self.get_parameter("pos_min").value)
        pos_max = _as_float_list(self.get_parameter("pos_max").value)
        vel_max = _as_float_list(self.get_parameter("vel_max").value)
        startup = _as_float_list(self.get_parameter("startup_pose").value)
        warn_margin = _as_float_list(self.get_parameter("warn_pos_margin").value)
        warn_near = [bool(v) for v in self.get_parameter("warn_near_limit").value]
        warn_pose = [bool(v) for v in self.get_parameter("warn_pose_deviation_joint").value]

        for label, values in (("pos_min", pos_min), ("pos_max", pos_max),
                              ("vel_max", vel_max), ("startup_pose", startup),
                              ("warn_pos_margin", warn_margin),
                              ("warn_near_limit", warn_near),
                              ("warn_pose_deviation_joint", warn_pose)):
            if len(values) != len(names):
                raise RuntimeError(
                    f"参数 {label} 长度 {len(values)} 与 joint_names({len(names)}) 不一致："
                    "safety_params.yaml 可能是旧版本，先跑 ./scripts/sync_config.py"
                )

        self.limits = SafetyLimits(
            names=names,
            pos_min=pos_min,
            pos_max=pos_max,
            vel_max=vel_max,
            startup=startup,
            warn_pos_margin=warn_margin,
            warn_near_limit=warn_near,
            warn_pose_deviation_joint=warn_pose,
            warn_vel_ratio=float(self.get_parameter("warn_vel_ratio").value),
            warn_on_pose_deviation=bool(self.get_parameter("warn_on_pose_deviation").value),
            warn_pose_threshold=float(self.get_parameter("warn_pose_threshold").value),
        )

        self.feedback_timeout = float(self.get_parameter("feedback_timeout_sec").value)
        self.estop_on_timeout = bool(self.get_parameter("estop_on_feedback_timeout").value)
        self.startup_grace = float(self.get_parameter("startup_grace_sec").value)
        self.latch = bool(self.get_parameter("latch").value)
        self.deactivate_hardware = bool(self.get_parameter("deactivate_hardware").value)
        self.hardware_component = str(self.get_parameter("hardware_component").value)
        self.service_timeout = float(self.get_parameter("service_timeout_sec").value)
        self.service_discovery_timeout = float(self.get_parameter("service_discovery_timeout_sec").value)
        self.deactivate_list = list(self.get_parameter("deactivate_controllers").value)

        # ---------------- 状态 ----------------
        self._positions: dict[str, float] = {}
        self._velocities: dict[str, float] = {}
        self._last_rx: float | None = None
        self._boot_time = time.monotonic()
        self._estop_active = False          # 急停已触发并锁存
        self._estop_reasons: list[str] = []
        self._actions_started = False
        self._actions_done = False
        self._evaluation = Evaluation(WARNING, ["等待首帧 /joint_states"])
        self._last_published: Evaluation | None = None

        # ---------------- 通信 ----------------
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        status_topic = str(self.get_parameter("status_topic").value)
        self.status_pub = self.create_publisher(DiagnosticStatus, status_topic, status_qos)

        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.joint_sub = self.create_subscription(
            JointState, joint_states_topic, self._on_joint_states,
            int(self.get_parameter("joint_state_depth").value)
        )

        self.clear_srv = self.create_service(
            Trigger, str(self.get_parameter("clear_service").value), self._on_clear_estop
        )

        self.list_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.hw_state_client = self.create_client(
            SetHardwareComponentState, "/controller_manager/set_hardware_component_state"
        )

        # ---------------- 定时器 ----------------
        eval_hz = max(float(self.get_parameter("minimum_evaluate_rate_hz").value), float(self.get_parameter("evaluate_rate_hz").value))
        pub_hz = max(float(self.get_parameter("minimum_publish_rate_hz").value), float(self.get_parameter("publish_rate_hz").value))
        self.eval_timer = self.create_timer(1.0 / eval_hz, self._evaluate_tick)
        self.pub_timer = self.create_timer(1.0 / pub_hz, self._publish_tick)

        self.get_logger().info(
            f"安全监控已启动：监控 {len(names)} 个关节（{', '.join(names)}），"
            f"评估 {eval_hz:.0f} Hz，状态话题 {status_topic}，"
            f"急停动作={'全臂失能' if self.deactivate_hardware else '仅停控制器'}"
        )
        summary = "，".join(
            f"{n}[{self.limits.pos_min[i]:+.2f},{self.limits.pos_max[i]:+.2f}]"
            f"/v{self.limits.vel_max[i]:.2f}"
            for i, n in enumerate(self.limits.names)
        )
        self.get_logger().info(f"限位/速度上限：{summary}")

    # ------------------------------------------------------------------ 回调
    def _on_joint_states(self, msg: JointState) -> None:
        self._positions = dict(zip(msg.name, msg.position))
        if msg.velocity and len(msg.velocity) == len(msg.name):
            self._velocities = dict(zip(msg.name, msg.velocity))
        else:
            self._velocities = {}
        self._last_rx = time.monotonic()
        # 每来一帧立刻评估，不等定时器：
        # 一个异常的反馈帧（例如编码器跳变）应该马上被判出来，而不是被下一帧 200 Hz 的
        # 正常数据覆盖掉。定时器只负责"断流超时"这类时间相关判定。
        self._evaluate_now()

    def _feedback_age(self) -> float | None:
        if self._last_rx is None:
            return None
        return time.monotonic() - self._last_rx

    # ------------------------------------------------------------ 状态评估
    def _evaluate_tick(self) -> None:
        """定时器入口：只做时间相关判定（断流/首帧），数据判定在回调里逐帧做。"""
        if self._estop_active and self.latch:
            return
        self._evaluate_now()

    def _evaluate_now(self) -> None:
        if self._estop_active and self.latch:
            # 锁存期间不重新判定，保持急停态（直到人工解除）
            return

        age = self._feedback_age()
        if age is None:
            in_grace = time.monotonic() - self._boot_time < self.startup_grace
            evaluation = Evaluation(
                WARNING, ["尚未收到 " + str(self.get_parameter("joint_states_topic").value)
                          + ("（启动宽限期内）" if in_grace else "")]
            )
        elif age > self.feedback_timeout:
            reasons = [f"/joint_states 已 {age:.2f}s 没有更新（阈值 {self.feedback_timeout:.2f}s）"]
            if self.estop_on_timeout:
                evaluation = Evaluation(EMERGENCY_STOP, reasons)
            else:
                evaluation = Evaluation(WARNING, reasons)
        else:
            evaluation = evaluate(self._positions, self._velocities, self.limits)

        previous_state = self._evaluation.state
        self._evaluation = evaluation

        if evaluation.state == EMERGENCY_STOP and not self._estop_active:
            self._enter_estop(evaluation)
        elif evaluation.state != previous_state:
            # 注意：不能在同一个调用点用变量切换 warn/info ——
            # rclpy 的 rcutils_logger 会缓存每个调用点的 severity，
            # 同一行既 warn 又 info 会抛 "Logger severity cannot be changed between calls"。
            if evaluation.state == WARNING:
                self.get_logger().warn(f"安全状态 {previous_state} → {evaluation.message}")
            else:
                self.get_logger().info(f"安全状态 {previous_state} → {evaluation.message}")
            # 状态一变立刻发，不等 20 Hz 保活 tick：
            # 否则短暂出现的 WARNING 可能在两次 tick 之间就被下一个正常帧覆盖掉。
            self._publish_tick(force=True)

    def _enter_estop(self, evaluation: Evaluation) -> None:
        self._estop_active = True
        self._estop_reasons = list(evaluation.reasons)
        self.get_logger().error(
            "🛑 急停触发：" + "；".join(evaluation.reasons) + " → 停止控制器并失能硬件"
        )
        self._publish_tick(force=True)
        self._start_estop_actions()

    # ------------------------------------------------------------ 状态发布
    def _publish_tick(self, force: bool = False) -> None:
        evaluation = self._evaluation
        if self._estop_active:
            reasons = ["急停已锁存：" + "；".join(self._estop_reasons)]
            if evaluation.state != EMERGENCY_STOP:
                reasons += list(evaluation.reasons)
            evaluation = Evaluation(EMERGENCY_STOP, reasons, evaluation.joints)
        if not force and self._last_published is not None:
            if (evaluation.state == self._last_published.state
                    and evaluation.reasons == self._last_published.reasons):
                return  # 状态没变就不刷（话题是 transient_local，订阅者随时能拿到）

        msg = DiagnosticStatus()
        msg.name = "b601_arm"
        msg.hardware_id = "b601_safety_monitor"
        _set_diag_level(msg, evaluation.level)
        msg.message = evaluation.message
        values = [
            KeyValue(key="state", value=evaluation.state),
            KeyValue(key="estop_latched", value="true" if self._estop_active else "false"),
            KeyValue(key="estop_actions_done",
                     value="true" if self._actions_done else "false"),
        ]
        for name, detail in evaluation.joints.items():
            pos = detail.get("position")
            vel = detail.get("velocity")
            values.append(KeyValue(
                key=name,
                value=("n/a" if pos is None else f"{pos:+.4f}")
                      + " / " + f"{(vel or 0.0):+.4f}"
                      + f" (限 [{detail['min']:+.2f},{detail['max']:+.2f}])",
            ))
        if self._estop_reasons:
            values.append(KeyValue(key="estop_reason", value="；".join(self._estop_reasons)))
        msg.values = values
        self.status_pub.publish(msg)
        self._last_published = evaluation

    # ------------------------------------------------------- 急停动作（异步链）
    def _start_estop_actions(self) -> None:
        if self._actions_started:
            return
        self._actions_started = True
        self._run_step_list_controllers()

    def _run_step_list_controllers(self) -> None:
        if not self.list_client.wait_for_service(timeout_sec=self.service_discovery_timeout):
            self.get_logger().warn(
                "拿不到 /controller_manager/list_controllers，按配置列表停控制器"
            )
            self._run_step_switch(self.deactivate_list)
            return
        future = self.list_client.call_async(ListControllers.Request())
        future.add_done_callback(self._on_list_controllers)

    def _on_list_controllers(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"list_controllers 失败（{exc}），按配置列表停控制器")
            self._run_step_switch(self.deactivate_list)
            return
        active = [c.name for c in response.controller if c.state == "active"]
        self.get_logger().info("当前 active 控制器：" + (", ".join(active) or "无"))
        if self.deactivate_list and ALL_CONTROLLERS not in self.deactivate_list:
            targets = [n for n in active if n in self.deactivate_list]
        else:
            targets = active
        self._run_step_switch(targets)

    def _run_step_switch(self, targets: list[str]) -> None:
        if not targets:
            self.get_logger().warn("没有 active 控制器可停（可能已经停了）")
            self._run_step_hardware()
            return
        if not self.switch_client.wait_for_service(timeout_sec=self.service_discovery_timeout):
            self.get_logger().error(
                "拿不到 /controller_manager/switch_controller，控制器仍在运行！"
            )
            self._run_step_hardware()
            return
        request = SwitchController.Request()
        request.activate_controllers = []
        request.deactivate_controllers = list(targets)
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = False
        request.timeout.sec = int(self.service_timeout)
        future = self.switch_client.call_async(request)
        future.add_done_callback(lambda f: self._on_switch(f, targets))

    def _on_switch(self, future, targets) -> None:
        try:
            ok = bool(future.result().ok)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"switch_controller 调用失败：{exc}")
            ok = False
        if ok:
            self.get_logger().info(f"已停用控制器：{', '.join(targets)}")
        else:
            self.get_logger().error(f"停用失败：{', '.join(targets)}")
        self._run_step_hardware()

    def _run_step_hardware(self) -> None:
        if not self.deactivate_hardware:
            self._finish_actions("按配置跳过硬件失能")
            return
        if not self.hw_state_client.wait_for_service(timeout_sec=self.service_discovery_timeout):
            self.get_logger().error(
                "拿不到 /controller_manager/set_hardware_component_state —— "
                "硬件没有失能，电机可能仍在出力！请立即断 24 V 电源。"
            )
            self._finish_actions("硬件失能服务不可用")
            return
        request = SetHardwareComponentState.Request()
        request.name = self.hardware_component
        request.target_state.id = LifecycleState.PRIMARY_STATE_INACTIVE
        request.target_state.label = "inactive"
        future = self.hw_state_client.call_async(request)
        future.add_done_callback(self._on_hardware_state)

    def _on_hardware_state(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"set_hardware_component_state 调用失败：{exc}")
            self._finish_actions("硬件失能失败")
            return
        if response.ok:
            self.get_logger().error(
                f"硬件 {self.hardware_component} 已置为 {response.state.label}："
                "电机已失能（DmSystemHardware::on_deactivate → disable_all）"
            )
            self._finish_actions("全臂已失能")
        else:
            self.get_logger().error(
                f"硬件失能被拒绝（当前 {response.state.label}）：请立即断 24 V 电源"
            )
            self._finish_actions("硬件失能被拒绝")

    def _finish_actions(self, note: str) -> None:
        self._actions_done = True
        self.get_logger().warn(
            f"急停动作结束（{note}）。解除前请确认现场安全，"
            f"然后调用服务 {self.get_parameter('clear_service').value}，"
            "再人工重新启动控制器。"
        )
        self._publish_tick(force=True)

    # ------------------------------------------------------- 解除急停（人工）
    def _on_clear_estop(self, _request, response):
        if not self._estop_active:
            response.success = False
            response.message = "当前不是急停状态，无需解除"
            return response
        # 必须确认实测已经回到安全范围，否则拒绝解除（避免"清一下就好了"的手感）
        current = evaluate(self._positions, self._velocities, self.limits)
        if current.state == EMERGENCY_STOP:
            response.success = False
            response.message = "实测仍越限，拒绝解除：" + "；".join(current.reasons)
            self.get_logger().error(response.message)
            return response
        self._estop_active = False
        self._actions_started = False
        self._actions_done = False
        self._estop_reasons = []
        self._evaluation = current
        self._publish_tick(force=True)
        response.success = True
        response.message = (
            "已解除急停锁存。控制器与硬件**不会**自动恢复，请确认现场安全后手动重新启动。"
        )
        self.get_logger().warn(response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SafetyMonitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
