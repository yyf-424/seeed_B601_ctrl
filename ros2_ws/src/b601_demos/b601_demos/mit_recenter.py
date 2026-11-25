"""Publish the B601's staged parking sequence to the existing motor-side MIT controller."""

from math import copysign, isfinite
from threading import Condition, Event, Lock
from time import monotonic

from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from b601_description.project_config import load_project_config
from b601_demos.trajectory_utils import mit_recenter_tolerances, recenter_position_bounds


RECENTER_SERVICE = "/b601_mit_recenter/recenter"


class MitRecenter(Node):
    def __init__(self):
        super().__init__("b601_mit_recenter")
        config_file = self.declare_parameter("project_config_file", "").value
        project = load_project_config(config_file or None)
        robot = project["robot"]
        park = project["recenter"]
        mit_park = park["mit"]
        check_collisions = self.declare_parameter("check_collisions", mit_park["check_collisions"]).value
        safety = project["safety"]
        plan = project["validation"]["mit_five_poses"]
        self.joints = robot["arm_joints"]
        self.gripper = robot["gripper_joint"]
        self.mimic = robot["mimic_joint"]
        self.j4 = self.joints.index("joint4")
        self.other_axes = tuple(i for i in range(len(self.joints)) if i != self.j4)
        self.home = tuple(park["joints"][name] for name in self.joints)
        self.bounds = [(project["joints"][name]["min_pos"], project["joints"][name]["max_pos"])
                       for name in self.joints]
        self.lift_tracking_tolerance = mit_park["joint4_lift_tracking_tolerance_rad"]
        self.tolerances = mit_recenter_tolerances(project)
        self.home_bounds = recenter_position_bounds(self.home, self.tolerances, self.bounds)
        self.clearance = mit_park["joint4_clearance_rad"]
        self.minimum_lift = mit_park["joint4_minimum_lift_rad"]
        if not self.bounds[self.j4][0] < self.clearance < 0 or self.minimum_lift <= 0:
            raise ValueError("J4 归中预置参数无效")
        self.slew = project["target_slew"]["revolute"]
        self.phase_timeout = mit_park["phase_timeout_sec"]
        self.feedback_timeout = safety["feedback_timeout_sec"]
        self.settle = mit_park["settle_window_sec"]
        self.correction_delay = mit_park["correction_wait_sec"]
        self.state_check_timeout = plan["state_check_timeout_sec"]
        self.collision_samples = plan["hardware_retest"]["collision_samples_per_transit"]

        self.changed = Condition()
        self.running = Lock()
        self.latest = None
        self.safety = None
        self.pub = self.create_publisher(JointState, "/b601_mit_arm/joint_target", 10)
        self.create_subscription(
            JointState, safety["joint_states_topic"], self.on_feedback, qos_profile_sensor_data)
        self.create_subscription(
            DiagnosticStatus, safety["status_topic"], self.on_safety,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.collision_client = None
        if check_collisions:
            from moveit_msgs.srv import GetStateValidity

            self.collision_client = self.create_client(GetStateValidity, "/check_state_validity")
        self.service_group = ReentrantCallbackGroup()
        self.create_service(
            Trigger, RECENTER_SERVICE, self.recenter, callback_group=self.service_group)
        self.startup_timer = None
        if self.declare_parameter("recenter_on_startup", False).value:
            timeout = plan["moveit_timeout_sec"] if check_collisions else plan["joint_state_timeout_sec"]
            self.startup_deadline = monotonic() + timeout
            self.startup_timer = self.create_timer(
                1.0 / safety["publish_rate_hz"], self.startup_recenter,
                callback_group=MutuallyExclusiveCallbackGroup())

    def startup_recenter(self):
        try:
            self.feedback()
            if self.collision_client is not None and not self.collision_client.service_is_ready():
                raise RuntimeError("MoveIt 碰撞检查未就绪")
        except RuntimeError as error:
            stopped = self.safety is not None and self.safety.level == DiagnosticStatus.ERROR
            if not stopped and monotonic() < self.startup_deadline:
                return
            self.get_logger().error(f"启动归中失败：{error}")
            self.startup_timer.cancel()
            self.startup_timer = None
            return

        self.startup_timer.cancel()
        try:
            self.run_sequence()
            self.get_logger().info("启动归中成功：六轴已归中")
        except RuntimeError as error:
            self.get_logger().error(f"启动归中失败：{error}")
        finally:
            self.startup_timer = None

    def on_feedback(self, message):
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
            return
        positions = dict(zip(message.name, message.position))
        if not all(name in positions and isfinite(positions[name]) for name in self.joints):
            return
        with self.changed:
            self.latest = (monotonic(), tuple(positions[name] for name in self.joints),
                           positions.get(self.gripper))
            self.changed.notify_all()

    def on_safety(self, message):
        with self.changed:
            self.safety = message
            self.changed.notify_all()

    def feedback(self):
        with self.changed:
            if not rclpy.ok():
                raise RuntimeError("ROS 已停止")
            if self.safety is None or self.safety.level not in (DiagnosticStatus.OK, DiagnosticStatus.WARN):
                status = self.safety.message if self.safety is not None else "未收到安全状态"
                raise RuntimeError(f"安全状态不允许继续归中：{status}")
            if self.latest is None or monotonic() - self.latest[0] >= self.feedback_timeout:
                raise RuntimeError("未收到新鲜的六轴位置反馈")
            if not self.pub.get_subscription_count():
                raise RuntimeError("MIT 位置目标话题没有控制器订阅")
            return self.latest

    def at_home(self, positions, axes):
        return all(self.home_bounds[i][0] <= positions[i] <= self.home_bounds[i][1] for i in axes)

    def bounded(self, index, angle):
        low, high = self.bounds[index]
        return max(low, min(angle, high))

    def check_path(self, before, target, gripper, deadline):
        client = self.collision_client
        if not client.service_is_ready() or gripper is None or not isfinite(gripper):
            raise RuntimeError("MoveIt 碰撞检查或夹爪反馈未就绪")
        deltas = [end - start for start, end in zip(before, target)]
        extent = max(map(abs, deltas))
        for sample in range(self.collision_samples):
            self.feedback()
            travel = extent * sample / (self.collision_samples - 1)
            q = [start + copysign(min(abs(delta), travel), delta)
                 for start, delta in zip(before, deltas)]
            request = client.srv_type.Request()
            request.robot_state.joint_state = JointState(
                name=[*self.joints, self.gripper, self.mimic], position=[*q, gripper, -gripper])
            done = Event()
            future = client.call_async(request)
            future.add_done_callback(lambda _, event=done: event.set())
            remaining = min(self.state_check_timeout, deadline - monotonic())
            if remaining <= 0 or not done.wait(remaining):
                future.cancel()
                raise RuntimeError("MoveIt 碰撞检查超时")
            result = future.result()
            if result is None or not result.valid:
                raise RuntimeError("归中目标路径未通过 MoveIt 碰撞检查")

    def publish(self, target, deadline):
        if any(angle != self.bounded(i, angle) for i, angle in enumerate(target)):
            raise RuntimeError("归中目标超出整机关节限位")
        _, before, gripper = self.feedback()
        if self.collision_client is not None:
            self.check_path(before, target, gripper, deadline)
        with self.changed:
            _, before, _ = self.feedback()
            stamp = monotonic()
            if stamp >= deadline:
                raise RuntimeError("归中阶段已超时")
            self.pub.publish(JointState(name=self.joints, position=list(target)))
        ramp_time = max(abs(end - start) for start, end in zip(before, target)) / self.slew
        return stamp, ramp_time

    def wait_for_position(self, after, deadline, reached):
        in_band_since = None
        previous = after
        with self.changed:
            while True:
                stamp, positions, _ = self.feedback()
                if stamp > previous:
                    previous = stamp
                    if reached(positions):
                        in_band_since = stamp if in_band_since is None else in_band_since
                        if stamp - in_band_since >= self.settle:
                            return True, positions
                    else:
                        in_band_since = None
                remaining = deadline - monotonic()
                if remaining <= 0:
                    return False, positions
                self.changed.wait(min(remaining, self.feedback_timeout))

    def move(self, label, target, reached, correction_axes=(), reference=None):
        deadline = monotonic() + self.phase_timeout
        target = list(target)
        correction_reference = self.home if reference is None else reference
        self.get_logger().info(f"归中阶段 {label}")
        try:
            stamp, ramp_time = self.publish(target, deadline)
            first_check = min(deadline, stamp + ramp_time + self.correction_delay)
            reached_target, actual = self.wait_for_position(stamp, first_check, reached)
            if reached_target:
                return target
            corrected = target.copy()
            for i in correction_axes:
                error = correction_reference[i] - actual[i]
                tolerance = self.tolerances[i] if reference is None else self.lift_tracking_tolerance
                if abs(error) > tolerance:
                    corrected[i] = self.bounded(i, target[i] + error)
            if corrected != target:
                stamp, _ = self.publish(corrected, deadline)
            reached_target, actual = self.wait_for_position(stamp, deadline, reached)
            if reached_target:
                return corrected
            raise RuntimeError(f"位置未到位，实测 {list(round(q, 4) for q in actual)} rad")
        except RuntimeError as error:
            raise RuntimeError(f"{label}：{error}") from error

    def run_sequence(self):
        _, start, _ = self.feedback()
        ranges = range(len(self.joints))
        lifted = list(start)
        if start[self.j4] > self.clearance:
            required = min(self.clearance, start[self.j4] - self.minimum_lift)
            lifted[self.j4] = self.bounded(self.j4, required - self.lift_tracking_tolerance)
            lifted = self.move(
                "J4 上抬", lifted, lambda q: q[self.j4] <= required, (self.j4,), lifted)

        other_home = list(self.home)
        other_home[self.j4] = lifted[self.j4]
        # Recheck clearance at the handoff even when J4 started inside the raised range.
        if self.feedback()[1][self.j4] > self.clearance:
            raise RuntimeError("J4 实测尚未进入归中预置角，未发送其他关节目标")
        at_intermediate = self.move(
            "其余关节归中", other_home,
            lambda q: q[self.j4] <= self.clearance and self.at_home(q, self.other_axes),
            self.other_axes)
        at_intermediate[self.j4] = self.home[self.j4]
        self.move("J4 回落", at_intermediate, lambda q: self.at_home(q, ranges), (self.j4,))

    def recenter(self, _request, response):
        if self.startup_timer is not None or not self.running.acquire(blocking=False):
            response.message = "已有归中调用正在执行"
            return response
        try:
            self.run_sequence()
            response.success = True
            response.message = "六轴已归中"
        except RuntimeError as error:
            response.message = str(error)
            self.get_logger().warning(response.message)
        finally:
            self.running.release()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MitRecenter()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
