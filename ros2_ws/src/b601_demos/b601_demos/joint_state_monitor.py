#!/usr/bin/env python3
"""joint_state_monitor.py —— 节点 b601_joint_state_monitor

角色：只读观察者。订阅标准 /joint_states，按固定频率把关节角打成表格。
这是排查“手臂到底去哪了”的第一件工具，也是本项目**唯一不依赖
ros2_control / MoveIt 的节点**（只用到 rclpy + sensor_msgs），
所以模型和驱动还不通时，它可以先用来看别的程序发出来的状态。

接口：
    订阅  /joint_states          sensor_msgs/JointState
    发布  无
    服务  无
    参数  topic (str, 默认 /joint_states)
          rate_hz (double, 默认 1.0)

用法：
    ros2 run b601_demos joint_state_monitor
    ros2 run b601_demos joint_state_monitor --ros-args -p rate_hz:=5.0
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from b601_description.project_config import load_project_config

PROJECT = load_project_config()
SETTINGS = PROJECT["nodes"]["joint_state_monitor"]

ARM_JOINTS = tuple(PROJECT["robot"]["arm_joints"])


class JointStateMonitor(Node):
    """把 /joint_states 定时打印成表格的极简订阅者。"""

    def __init__(self):
        super().__init__("b601_joint_state_monitor")

        self.declare_parameter("topic", "/joint_states")
        self.declare_parameter("rate_hz", SETTINGS["rate_hz"])

        topic = self.get_parameter("topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)

        # 最新一帧状态；没收到之前 self.latest 为 None
        self.latest = None
        self.message_count = 0

        self.subscription = self.create_subscription(
            JointState, topic, self._on_joint_state, 10
        )
        self.timer = self.create_timer(1.0 / max(rate_hz, SETTINGS["minimum_rate_hz"]), self._report)

        self.get_logger().info(
            f"b601_joint_state_monitor 已启动：订阅 {topic}，每 {rate_hz:.1f} Hz 打印一次"
        )

    def _on_joint_state(self, msg: JointState):
        self.latest = msg
        self.message_count += 1

    def _report(self):
        if self.latest is None:
            self.get_logger().warn(
                "还没收到 /joint_states —— 检查 bringup 是否起了 joint_state_broadcaster",
                throttle_duration_sec=SETTINGS["warning_throttle_sec"],
            )
            return

        msg = self.latest
        # JointState 里 name 的顺序不保证，按名字取；缺的就显示 -
        values = dict(zip(msg.name, msg.position))

        cells = []
        for name in ARM_JOINTS:
            if name in values:
                rad = values[name]
                cells.append(f"{name}={rad:+.3f}rad({math.degrees(rad):+6.1f}°)")
            else:
                cells.append(f"{name}=--")
        gripper = values.get("finger_left")
        gripper_txt = f"gripper={gripper * 1000:.1f}mm" if gripper is not None else "gripper=--"

        self.get_logger().info(
            " | ".join(cells) + f" | {gripper_txt} | 共收到 {self.message_count} 帧"
        )


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
