#!/usr/bin/env python3
"""dm_moveit_step.py —— 真机上的 MoveIt 小步移动（默认只规划不执行）

为什么不用 b601_demos/moveit_pose_goal：
  那个脚本要求手填绝对位姿，真机上很容易填到一个"够不着/会撞"的点。
  这里先用 TF 读**当前末端位姿**，再叠加一个小偏移（默认 z +2 cm），
  保证目标就在手边，且会先打印规划出的轨迹明细（点数/时长/各关节最大增量）。

⚠ 为什么 --execute 不是"再规划一次然后执行"：
   OMPL 的 RRTConnect 是随机采样规划器，**同一个目标每次解出的路径都不同**。
   实测：同一次 2 cm 抬升，预览得到 8 点/0.70s/最大增量 4.2°，
   重规划执行时却给出 35 点/3.34s/最大增量 21.8°。
   所以本脚本的做法是：**规划一次（plan_only）→ 检查 → 用 /execute_trajectory
   执行同一条轨迹**，保证"你看到的"就是"执行的"。

用法（前提：bringup 已在跑 + move_group 已起）：
    ./scripts/dm_moveit_step.py                       # plan_only，z +2cm
    ./scripts/dm_moveit_step.py --dz -0.02            # 往下 2cm（只规划）
    ./scripts/dm_moveit_step.py --execute --dz 0.02   # 真的执行
    ./scripts/dm_moveit_step.py --execute --dz 0.02 --scaling 0.1   # 更慢

安全：默认 plan_only=true。轨迹里只要有任一关节增量超过 --max-delta（默认 0.5 rad），
      即使加了 --execute 也会拒绝执行并打印原因。
"""

import argparse
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume, Constraints, MotionPlanRequest, MoveItErrorCodes,
    OrientationConstraint, PlanningOptions, PositionConstraint,
)
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration as MsgDuration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

from project_config import load_project_config

PROJECT = load_project_config()
SETTINGS = PROJECT["tools"]["dm_moveit_step"]


def traj_to_dict(rt):
    """把 RobotTrajectory 存成 JSON（不引入外部依赖，方便人工核对）。"""
    jt = rt.joint_trajectory
    return {
        "joint_names": list(jt.joint_names),
        "points": [
            {"p": [float(x) for x in pt.positions],
             "v": [float(x) for x in pt.velocities],
             "t": pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9}
            for pt in jt.points
        ],
    }


def dict_to_traj(d):
    """从 JSON 还原 RobotTrajectory，用于 /execute_trajectory。"""
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    rt = RobotTrajectory()
    jt = rt.joint_trajectory
    jt.joint_names = d["joint_names"]
    for item in d["points"]:
        pt = JointTrajectoryPoint()
        pt.positions = item["p"]
        if item.get("v"):
            pt.velocities = item["v"]
        sec = int(item["t"])
        # 注意：轨迹点里的时间戳是 **消息类型** builtin_interfaces/Duration，
        # 不是 rclpy.duration.Duration（后者参数叫 seconds/nanoseconds）。
        pt.time_from_start = MsgDuration(sec=sec, nanosec=int(round((item["t"] - sec) * 1e9)))
        jt.points.append(pt)
    return rt


class MoveItStep(Node):
    def __init__(self):
        super().__init__("b601_dm_moveit_step")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.client = ActionClient(self, MoveGroup, "/move_action")
        # 复用同一条轨迹执行：这一步是"预览即执行"的关键
        self.exec_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

    def lookup_ee(self, base="base_link", tip="end_link", timeout=SETTINGS["tf_timeout_sec"]):
        import time
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                tf = self.buffer.lookup_transform(base, tip, Time())
                p = Pose()
                p.position.x = tf.transform.translation.x
                p.position.y = tf.transform.translation.y
                p.position.z = tf.transform.translation.z
                p.orientation = tf.transform.rotation
                return p
            except Exception:  # noqa: BLE001 - TF 还没到就是等
                rclpy.spin_once(self, timeout_sec=SETTINGS["poll_sec"])
        return None

    def build_goal(self, target: Pose, frame: str, tol: float, ang_tol: float,
                   plan_only: bool, scaling: float):
        position = PositionConstraint()
        position.header.frame_id = frame
        position.link_name = "end_link"
        position.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(tol)]
        vol = BoundingVolume()
        vol.primitives.append(sphere)
        center = Pose()
        center.position = target.position
        center.orientation.w = 1.0
        vol.primitive_poses.append(center)
        position.constraint_region = vol

        orientation = OrientationConstraint()
        orientation.header.frame_id = frame
        orientation.link_name = "end_link"
        orientation.orientation = target.orientation
        orientation.absolute_x_axis_tolerance = float(ang_tol)
        orientation.absolute_y_axis_tolerance = float(ang_tol)
        orientation.absolute_z_axis_tolerance = float(ang_tol)
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)

        req = MotionPlanRequest()
        req.group_name = "arm"
        req.goal_constraints.append(constraints)
        req.num_planning_attempts = 10
        req.allowed_planning_time = SETTINGS["planning_time_sec"]
        req.max_velocity_scaling_factor = float(scaling)
        req.max_acceleration_scaling_factor = float(scaling)
        req.start_state.is_diff = True

        opts = PlanningOptions()
        opts.plan_only = bool(plan_only)
        opts.planning_scene_diff.is_diff = True
        opts.planning_scene_diff.robot_state.is_diff = True

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = opts
        return goal


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dx", type=float, default=SETTINGS["dx"])
    ap.add_argument("--dy", type=float, default=SETTINGS["dy"])
    ap.add_argument("--dz", type=float, default=SETTINGS["dz"], help="默认抬高 2 cm")
    ap.add_argument("--execute", action="store_true",
                    help="规划后立刻执行同一条轨迹（不重规划）")
    ap.add_argument("--save", default="",
                    help="把规划结果存成 JSON（默认存到 .verify/plan_<时间>.json）")
    ap.add_argument("--execute-from", default="",
                    help="直接执行某个 plan JSON（不规划），用于\"预览→确认→执行\"")
    ap.add_argument("--scaling", type=float, default=SETTINGS["scaling"], help="速度/加速度缩放")
    ap.add_argument("--tol", type=float, default=SETTINGS["tol"])
    ap.add_argument("--ang-tol", type=float, default=SETTINGS["ang_tol"])
    ap.add_argument("--max-delta", type=float, default=SETTINGS["max_delta"],
                    help="任一关节规划增量超过此值就拒绝执行（rad）")
    ap.add_argument("--frame", default="base_link")
    args = ap.parse_args()

    rclpy.init()
    node = MoveItStep()
    try:
        if args.execute_from:
            import json
            with open(args.execute_from, encoding="utf-8") as f:
                d = json.load(f)
            rt = dict_to_traj(d)
            total = d["points"][-1]["t"] if d["points"] else 0.0
            deltas = [abs(b - a) for a, b in zip(d["points"][0]["p"], d["points"][-1]["p"])] if d["points"] else []
            print(f"载入 {args.execute_from}: {len(d['points'])} 点 / {total:.2f}s / "
                  f"最大关节增量 {max(deltas) if deltas else 0:.3f} rad")
            if deltas and max(deltas) > args.max_delta:
                print(f"⚠ 超过 --max-delta {args.max_delta}，拒绝执行")
                return 1
            if not node.exec_client.wait_for_server(timeout_sec=SETTINGS["server_timeout_sec"]):
                print("❌ 等不到 /execute_trajectory")
                return 1
            exec_goal = ExecuteTrajectory.Goal()
            exec_goal.trajectory = rt
            send = node.exec_client.send_goal_async(exec_goal)
            rclpy.spin_until_future_complete(node, send, timeout_sec=SETTINGS["execution_request_timeout_sec"])
            handle = send.result()
            if handle is None or not handle.accepted:
                print("❌ 执行请求被拒绝")
                return 1
            fut = handle.get_result_async()
            rclpy.spin_until_future_complete(node, fut, timeout_sec=total + SETTINGS["replay_result_grace_sec"])
            ex = fut.result()
            if ex is None or ex.result.error_code.val != MoveItErrorCodes.SUCCESS:
                code = None if ex is None else ex.result.error_code.val
                print(f"❌ 执行失败 MoveItErrorCodes={code}")
                return 1
            print("\n✅ 已按文件里的轨迹执行完成（未重新规划）")
            return 0

        cur = node.lookup_ee(base=args.frame)
        if cur is None:
            print(f"❌ 5 秒内取不到 {args.frame} → end_link 的 TF（robot_state_publisher 在跑吗？）")
            return 1
        target = Pose()
        target.position.x = cur.position.x + args.dx
        target.position.y = cur.position.y + args.dy
        target.position.z = cur.position.z + args.dz
        target.orientation = cur.orientation
        print(f"当前末端 [{args.frame}]: ({cur.position.x:+.4f}, {cur.position.y:+.4f}, "
              f"{cur.position.z:+.4f})")
        print(f"目标末端:              ({target.position.x:+.4f}, {target.position.y:+.4f}, "
              f"{target.position.z:+.4f})  Δ=({args.dx:+.3f},{args.dy:+.3f},{args.dz:+.3f}) m")

        if not node.client.wait_for_server(timeout_sec=SETTINGS["server_timeout_sec"]):
            print("❌ 等不到 /move_action：先起 ros2 launch b601_moveit_config move_group.launch.py")
            return 1

        goal = node.build_goal(target, args.frame, args.tol, args.ang_tol,
                              True, args.scaling)  # 永远先只规划
        send = node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send, timeout_sec=SETTINGS["server_timeout_sec"])
        handle = send.result()
        if handle is None or not handle.accepted:
            print("❌ MoveGroup 拒绝请求")
            return 1

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=SETTINGS["result_timeout_sec"])
        res = result_future.result()
        if res is None:
            print("❌ 等结果超时")
            return 1
        code = res.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            print(f"❌ 规划/执行失败 MoveItErrorCodes={code}")
            return 1

        # ---- 轨迹明细 ----
        traj = res.result.planned_trajectory.joint_trajectory
        if not traj.points:
            print("⚠ 规划返回空轨迹（可能已在目标附近）")
            return 0
        names = list(traj.joint_names)
        first = list(traj.points[0].positions)
        last = list(traj.points[-1].positions)
        deltas = [abs(b - a) for a, b in zip(first, last)]
        dur = traj.points[-1].time_from_start
        total = dur.sec + dur.nanosec * 1e-9
        print(f"\n规划结果：{len(traj.points)} 点，总时长 {total:.2f}s，"
              f"缩放 {args.scaling:.2f}")
        for n, a, b, d in zip(names, first, last, deltas):
            print(f"  {n:<8} {a:+.4f} → {b:+.4f}   Δ={b - a:+.4f} rad ({(b - a) * 57.2958:+.1f}°)")

        worst = max(deltas) if deltas else 0.0
        if worst > args.max_delta:
            print(f"\n⚠ 最大关节增量 {worst:.3f} rad 超过阈值 {args.max_delta} → 拒绝执行")
            return 1

        # 存盘，供 --execute-from 精确复现
        import json
        import time as _t
        save_path = args.save or f".verify/plan_{_t.strftime('%Y%m%d_%H%M%S')}.json"
        with open(save_path, "w", encoding="utf-8") as f:
            json.dump(traj_to_dict(res.result.planned_trajectory), f, indent=1)
        print(f"\n轨迹已存盘：{save_path}")

        if not args.execute:
            print("✅ 规划成功（未执行）。确认这条路径没问题后，用**同一条轨迹**执行：")
            print(f"   ./scripts/dm_moveit_step.py --execute-from {save_path}")
            return 0

        # ---- 用刚规划出的同一条轨迹执行 ----
        if not node.exec_client.wait_for_server(timeout_sec=SETTINGS["execution_request_timeout_sec"]):
            print("❌ 等不到 /execute_trajectory")
            return 1
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = res.result.planned_trajectory
        send = node.exec_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(node, send, timeout_sec=SETTINGS["execution_request_timeout_sec"])
        handle = send.result()
        if handle is None or not handle.accepted:
            print("❌ 执行请求被拒绝")
            return 1
        fut = handle.get_result_async()
        rclpy.spin_until_future_complete(node, fut, timeout_sec=total + SETTINGS["execution_result_grace_sec"])
        ex = fut.result()
        if ex is None:
            print("❌ 执行超时")
            return 1
        if ex.result.error_code.val != MoveItErrorCodes.SUCCESS:
            print(f"❌ 执行失败 MoveItErrorCodes={ex.result.error_code.val}")
            return 1
        print("\n✅ 已执行上面这条轨迹（与预览完全一致）")
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n已中止")
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
