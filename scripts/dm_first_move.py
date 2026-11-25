#!/usr/bin/env python3
"""dm_first_move.py —— 真机的第一个运动：只动一个关节、幅度可调、慢慢走

为什么单独写、不用 b601_demos/move_to_joints：
  - 它会**自动读取当前真实关节角**作为起点，目标只在一个关节上叠加 delta，
    其它关节保持不动 —— 避免手填 target 时把别的关节也带偏。
  - 默认 delta=0.05 rad（2.9°）、duration=6 s（≈0.5°/s），比任何限速都慢得多，
    第一次上电时即使方向/零点有小偏差也来得及反应。
  - 结束时打印跟踪误差与最终实测值，用来判断"真闭环"是否成立。

用法（前提：另开终端已 `hardware:=dm dm_enable_motors:=true` 起了 bringup）：
    ./scripts/dm_first_move.py                        # joint1 +0.05 rad
    ./scripts/dm_first_move.py --joint joint2 --delta -0.05
    ./scripts/dm_first_move.py --delta -0.05          # 反向走回来

安全：任何一步失败都立刻停止，不做自动重试；如需中止按 Ctrl-C（会撤销 goal）。
"""

import argparse
import sys

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

from b601_demos.trajectory_utils import RECENTER_POSITIONS, RECENTER_TOLERANCE, at_recenter

from project_config import load_project_config

PROJECT = load_project_config()
SETTINGS = PROJECT["tools"]["dm_first_move"]

ARM_JOINTS = PROJECT["robot"]["arm_joints"]
def load_limits():
    """限位从统一真值读（唯一真值原则）：历史上这里抄过一份旧表
    （joint4 −1.87 / joint5 −1.57），比真值宽，和 URDF/MoveIt 打架。"""
    j = PROJECT["joints"]
    return {n: (float(j[n]["min_pos"]), float(j[n]["max_pos"])) for n in ARM_JOINTS}


LIMITS = load_limits()


class FirstMove(Node):
    def __init__(self, controller):
        super().__init__("b601_first_move")
        self.current = None
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        self.client = ActionClient(self, FollowJointTrajectory,
                                   f"/{controller}/follow_joint_trajectory")

    def _on_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(n in values for n in ARM_JOINTS):
            self.current = [values[n] for n in ARM_JOINTS]

    def wait_state(self, timeout=SETTINGS["state_timeout_sec"]):
        import time
        t0 = time.time()
        while self.current is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=SETTINGS["poll_sec"])
        return self.current is not None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default=ARM_JOINTS[0], choices=ARM_JOINTS)
    ap.add_argument("--delta", type=float, default=SETTINGS["delta"], help="增量（rad），带符号")
    target_mode = ap.add_mutually_exclusive_group()
    target_mode.add_argument("--set", default="",
                             help="多关节绝对目标，如 joint2=-0.3,joint3=-0.6")
    target_mode.add_argument("--recenter", action="store_true",
                             help="去校准停放位，并检查全部手臂关节的到位容差")
    ap.add_argument("--duration", type=float, default=SETTINGS["duration"], help="到位时间（s）")
    ap.add_argument("--controller", default=PROJECT["controllers"]["arm"]["name"])
    args = ap.parse_args()

    rclpy.init()
    node = FirstMove(args.controller)
    try:
        if not node.wait_state():
            print("❌ 5 秒内没收到 /joint_states：bringup 起来了吗？")
            return 1

        start = list(node.current)
        target = list(start)
        moved = []
        if args.recenter:
            target = list(RECENTER_POSITIONS)
            moved = list(range(len(ARM_JOINTS)))
        elif args.set:
            # 多关节绝对目标：只改给定关节，其余保持当前值
            for item in args.set.split(","):
                name, _, value = item.partition("=")
                name = name.strip()
                if name not in ARM_JOINTS:
                    print(f"❌ 未知关节 {name}")
                    return 1
                i = ARM_JOINTS.index(name)
                target[i] = float(value)
                moved.append(i)
        else:
            idx = ARM_JOINTS.index(args.joint)
            target[idx] = start[idx] + args.delta
            moved = [idx]

        # 逐关节校验软限位
        for i in range(len(ARM_JOINTS)):
            lo, hi = LIMITS[ARM_JOINTS[i]]
            # 不再额外内缩 0.02：真值限位本身已是"官方值与实测值的保守交集"，
            # 而校准后的归中位 j2=−0.0109 距上限只有 0.011 rad，内缩会把它判成越限。
            if not (lo <= target[i] <= hi):
                print(f"❌ 目标 {ARM_JOINTS[i]}={target[i]:+.4f} 超出软限位 [{lo}, {hi}]，拒绝发送")
                return 1

        print("起点: " + " ".join(f"{n}={v:+.4f}" for n, v in zip(ARM_JOINTS, start)))
        print("目标: " + " ".join(f"{n}={v:+.4f}" for n, v in zip(ARM_JOINTS, target)))
        for i in moved:
            print(f"     {ARM_JOINTS[i]}: {start[i]:+.4f} → {target[i]:+.4f} "
                  f"(Δ={target[i]-start[i]:+.4f} rad = {(target[i]-start[i]) * 57.2958:+.1f}°)")
        print(f"     用时 {args.duration:.1f}s；未列出的关节保持不动")

        if not node.client.wait_for_server(timeout_sec=SETTINGS["action_timeout_sec"]):
            print(f"❌ 等不到 action server /{args.controller}/follow_joint_trajectory")
            return 1

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        from trajectory_msgs.msg import JointTrajectoryPoint
        for t, pos in ((0.0, start), (args.duration, target)):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in pos]
            sec = int(t)
            pt.time_from_start = Duration(sec=sec, nanosec=int((t - sec) * 1e9))
            goal.trajectory.points.append(pt)
        goal.goal_time_tolerance.sec = SETTINGS["goal_time_tolerance_sec"]

        print("\n发送轨迹 ...（如果方向不对或异常，立刻 Ctrl-C / 断电）")
        send = node.client.send_goal_async(goal, feedback_callback=lambda f: None)
        rclpy.spin_until_future_complete(node, send, timeout_sec=SETTINGS["action_timeout_sec"])
        handle = send.result()
        if handle is None or not handle.accepted:
            print("❌ 轨迹被拒绝")
            return 1

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.duration + SETTINGS["result_grace_sec"])
        result = result_future.result()
        if result is None:
            print("❌ 等待结果超时")
            return 1

        code = result.result.error_code
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            print(f"❌ 执行失败 error_code={code} error_string={result.result.error_string!r}")
            return 1

        # 回读最终实测值，算跟踪误差
        node.current = None
        if node.wait_state(timeout=SETTINGS["final_state_timeout_sec"]):
            final = node.current
            err = [abs(a - b) for a, b in zip(final, target)]
            print("\n✅ 执行成功")
            print("最终实测: " + " ".join(f"{n}={v:+.4f}" for n, v in zip(ARM_JOINTS, final)))
            print(f"跟踪误差: 最大 {max(err):.4f} rad ({max(err) * 57.2958:.2f}°)")
            for i in moved:
                print(f"实际位移: {ARM_JOINTS[i]} {final[i] - start[i]:+.4f} rad "
                      f"（命令 {target[i] - start[i]:+.4f}）")
            if args.recenter:
                if not at_recenter(final):
                    print(f"❌ 归中最大误差 {max(err):.9g} rad，要求每轴 ≤ {RECENTER_TOLERANCE:g} rad")
                    return 1
                print(f"已归中：六轴误差均 ≤ {RECENTER_TOLERANCE:g} rad")
        else:
            if args.recenter:
                print("❌ 没读到最终状态，无法确认归中")
                return 1
            print("✅ 执行成功（但没读到最终状态）")
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n已中止")
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
