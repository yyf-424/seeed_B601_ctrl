#!/usr/bin/env python3
"""moveit_pose_goal.py —— 节点 b601_moveit_pose_goal

角色：这一层和上面两个演示节点完全不同 —— 指挥的是 MoveIt，不是控制器。
你把“末端要到哪里”告诉 move_group，它负责逆解、避碰、规划、时间参数化，
最后才把轨迹交给 controller。

接口：
    动作客户端  /move_action                      moveit_msgs/action/MoveGroup
    订阅        无（起点用 start_state.is_diff=true，由 move_group 自己取当前状态）
    参数        group       (str, 默认 arm)
                frame_id    (str, 默认 base_link)  目标位姿所在的坐标系
                x, y, z     (double, 默认 0.25, 0.00, 0.15)  目标位置 m
                roll, pitch, yaw (double, 默认 0.0, 1.57, 0.0) 目标姿态 rad
                tol         (double, 默认 0.01) 位置容差 m
                ang_tol     (double, 默认 0.1)  姿态容差 rad
                plan_only   (bool,   默认 false)

用法（3 个常用例子）：
    # 往正前方 25cm、高 15cm 处，末端朝下（实测可达）
    ros2 run b601_demos moveit_pose_goal
    # 只规划不执行，先看 RViz 里的路径
    ros2 run b601_demos moveit_pose_goal --ros-args -p plan_only:=true
    # 换个目标点
    ros2 run b601_demos moveit_pose_goal --ros-args -p x:=0.25 -p z:=0.20 -p y:=-0.1

前提：move_group 已启动（ros2 launch b601_moveit_config demo.launch.py）。
     没规划出来通常只有三个原因：目标不可达 / 姿态不可达 / 目标点在碰撞里。
"""

import math

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)

from b601_demos.trajectory_utils import spin_until
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


def rpy_to_quaternion(roll, pitch, yaw):
    """欧拉角 → 四元数（roll-pitch-yaw，等价于 tf2 的 RPY 约定）。

    B601 的 end_link 坐标系 x 轴指向指尖外侧，所以“工具朝下”通常是
    pitch=1.57（把 x 轴转到 -z 方向）。方向不对时优先怀疑这个。
    """
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


class MoveItPoseGoal(Node):
    def __init__(self):
        super().__init__("b601_moveit_pose_goal")

        self.declare_parameter("group", "arm")
        self.declare_parameter("frame_id", "base_link")
        # 默认点是用 /compute_ik 实测扫出来的可达点（工具朝下时 z 必须 ≲0.15 m，
        # 这个臂的展长比看上去短：0.32 + 0.30 m 连杆 + 0.155 m 末端偏置）
        self.declare_parameter("x", 0.25)
        self.declare_parameter("y", 0.00)
        self.declare_parameter("z", 0.15)
        self.declare_parameter("roll", 0.0)
        self.declare_parameter("pitch", 1.57)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("tol", 0.01)
        self.declare_parameter("ang_tol", 0.1)
        self.declare_parameter("plan_only", False)
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("velocity_scaling", 0.2)
        self.declare_parameter("acceleration_scaling", 0.2)

        self.client = ActionClient(self, MoveGroup, "/move_action")

    def _params(self):
        p = self.get_parameter
        return {
            "group": p("group").value,
            "frame_id": p("frame_id").value,
            "xyz": (p("x").value, p("y").value, p("z").value),
            "rpy": (p("roll").value, p("pitch").value, p("yaw").value),
            "tol": p("tol").value,
            "ang_tol": p("ang_tol").value,
            "plan_only": p("plan_only").value,
            "allowed_planning_time": p("allowed_planning_time").value,
            "velocity_scaling": p("velocity_scaling").value,
            "acceleration_scaling": p("acceleration_scaling").value,
        }

    def _build_goal(self) -> MoveGroup.Goal:
        cfg = self._params()
        x, y, z = cfg["xyz"]
        roll, pitch, yaw = cfg["rpy"]
        qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)

        # ---- 位置约束：一个以目标点为中心的球，半径 = 位置容差 ----
        position = PositionConstraint()
        position.header.frame_id = cfg["frame_id"]
        position.link_name = "end_link"          # arm 组的 tip link
        position.target_point_offset.x = 0.0
        position.target_point_offset.y = 0.0
        position.target_point_offset.z = 0.0
        position.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [cfg["tol"]]

        center = Pose()
        center.position.x = float(x)
        center.position.y = float(y)
        center.position.z = float(z)
        center.orientation.w = 1.0

        volume = BoundingVolume()
        volume.primitives.append(sphere)
        volume.primitive_poses.append(center)
        position.constraint_region = volume

        # ---- 姿态约束：每个轴允许的角度误差 ----
        orientation = OrientationConstraint()
        orientation.header.frame_id = cfg["frame_id"]
        orientation.link_name = "end_link"
        orientation.orientation.x = qx
        orientation.orientation.y = qy
        orientation.orientation.z = qz
        orientation.orientation.w = qw
        orientation.absolute_x_axis_tolerance = cfg["ang_tol"]
        orientation.absolute_y_axis_tolerance = cfg["ang_tol"]
        orientation.absolute_z_axis_tolerance = cfg["ang_tol"]
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)

        # ---- 规划请求 ----
        request = MotionPlanRequest()
        request.group_name = cfg["group"]
        request.goal_constraints.append(constraints)
        request.num_planning_attempts = 10
        request.allowed_planning_time = cfg["allowed_planning_time"]
        request.max_velocity_scaling_factor = cfg["velocity_scaling"]
        request.max_acceleration_scaling_factor = cfg["acceleration_scaling"]
        # 起点 = 机器人当前状态，不用自己填
        request.start_state.is_diff = True

        options = PlanningOptions()
        options.plan_only = bool(cfg["plan_only"])
        options.look_around = False
        options.replan = False
        options.replan_attempts = 0
        options.planning_scene_diff.is_diff = True
        options.planning_scene_diff.robot_state.is_diff = True

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = options
        return goal

    def run(self) -> int:
        cfg = self._params()
        x, y, z = cfg["xyz"]
        roll, pitch, yaw = cfg["rpy"]
        self.get_logger().info(
            f"目标位姿 [{cfg['frame_id']}] xyz=({x:.3f}, {y:.3f}, {z:.3f}) "
            f"rpy=({roll:.2f}, {pitch:.2f}, {yaw:.2f}) group={cfg['group']} "
            f"plan_only={cfg['plan_only']}"
        )

        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "等不到 /move_action 服务器 —— 先启动：ros2 launch b601_moveit_config demo.launch.py"
            )
            return 1

        send_future = self.client.send_goal_async(self._build_goal())
        if spin_until(self, send_future, 20.0, "发送 MoveGroup 目标") is None:
            return 1
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("MoveGroup 拒绝了请求")
            return 1

        self.get_logger().info("规划中 ...")
        plan_timeout = float(self.get_parameter("allowed_planning_time").value)
        result = spin_until(self, handle.get_result_async(), plan_timeout + 180.0,
                            "等待规划/执行结果")
        if result is None:
            return 1
        code = result.result.error_code.val

        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"❌ 失败，MoveItErrorCodes={code}（{_error_name(code)}）\n"
                "   常见原因：\n"
                "   - NO_IK_SOLUTION(-31)：目标点超出臂展，或姿态不可达\n"
                "   - INVALID_MOTION_PLAN(-16)/GOAL_IN_COLLISION(-12)：目标在碰撞里\n"
                "   - CONTROL_FAILED(-4)：轨迹下发后 controller 执行失败（看 controller 日志）\n"
                "   先用 RViz 的 MotionPlanning 面板把拖拽目标调到一个能到的位置，"
                "再把参数抄过来。"
            )
            return 1

        planned = result.result.planned_trajectory.joint_trajectory
        if planned.points:
            self.get_logger().info(
                f"✅ {'规划成功（plan_only，未执行）' if cfg['plan_only'] else '规划并执行成功'}："
                f"轨迹 {len(planned.points)} 点"
            )
        else:
            self.get_logger().info("✅ 成功（返回轨迹为空，可能是已在目标位姿）")
        return 0


def _error_name(code: int) -> str:
    for attr in dir(MoveItErrorCodes):
        if attr.isupper() and getattr(MoveItErrorCodes, attr) == code:
            return attr
    return "UNKNOWN"


def main(args=None):
    rclpy.init(args=args)
    node = MoveItPoseGoal()
    try:
        code = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        code = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
