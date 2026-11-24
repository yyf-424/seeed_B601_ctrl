"""logic.py —— 安全状态的纯计算（不 import rclpy，可单独跑测试）。

把「判定」和「ROS 通信」彻底分开，好处：
    · 判定逻辑可以用普通 python 单测（scripts/test_safety.py），不用起 ROS；
    · 改阈值不会牵动通信代码；
    · 出问题时要回答"为什么判成急停"，看这一个文件即可。

状态三档：
    NORMAL           正常：都在安全范围内，且（若启用）仍停留在启动位形附近
    WARNING          警告：仍在安全范围内，但接近限位 / 速度偏高 / 已离开启动位形 / 反馈不全
    EMERGENCY_STOP   急停：角度越限 或 角速度越限 —— 只有这两类会触发急停

设计约束（来自项目约定）：
    · 急停只看「角度 / 角速度」两类硬性越限，不掺入其它条件，避免误停；
    · 警告绝不下发任何停机动作，机械臂可以照常被驱动；
    · 反馈断流默认只警告（estop_on_feedback_timeout 可改成升级为急停）；
    · 「接近限位 / 离开启动位形」这些软警告可以逐关节关闭
      （夹爪开合是正常操作，不该一直挂着警告）。
"""

from __future__ import annotations

from dataclasses import dataclass, field

NORMAL = "NORMAL"
WARNING = "WARNING"
EMERGENCY_STOP = "EMERGENCY_STOP"

# 与 diagnostic_msgs/msg/DiagnosticStatus 的常量对齐（这里不 import 消息包，
# 保证本模块在没装 ROS 的环境里也能跑测试）
LEVEL_OK = 0
LEVEL_WARN = 1
LEVEL_ERROR = 2

STATE_LEVEL = {NORMAL: LEVEL_OK, WARNING: LEVEL_WARN, EMERGENCY_STOP: LEVEL_ERROR}

# 判定容差：浮点比较用的"物理噪声底"，不是数学 ε。
#   · 关节角单位 rad：1e-3 rad ≈ 0.057°，远小于任何真实安全余量（警告余量 0.12 rad）
#   · 夹爪单位 m：1e-3 m = 1 mm
# 为什么不能取 1e-6：2026-11-21 真机实测，夹爪停在 0.000（限位边界）时编码器量化
# 会报 ±1.9e-6，1e-6 的容差挡不住 → 判定"越限"→ 假急停把整条手臂失能。
# 编码器/ADC 的量化台阶（夹爪约 2e-6 m、关节约 2e-3 rad 量级）必须在容差里被吸收。
EPS = 1e-3


def _broadcast(value, n: int) -> list:
    """标量 → 长度 n 的列表（兼容"只配一个值"的写法）。"""
    if isinstance(value, (list, tuple)):
        return list(value)
    return [value] * n


@dataclass
class SafetyLimits:
    """一份冻结的阈值快照（由节点参数构造，运行期不变）。"""

    names: list[str]
    pos_min: list[float]
    pos_max: list[float]
    vel_max: list[float]
    startup: list[float]

    # 逐关节：距离限位多近开始警告（关节单位：rad 或 m）
    warn_pos_margin: float | list[float] | None = None
    # 逐关节：是否启用"接近限位"警告
    warn_near_limit: list[bool] = True
    # 逐关节：是否启用"离开启动位形"警告
    warn_pose_deviation_joint: list[bool] = True

    estop_pos_margin: float | None = None
    # 静止零位漂移容差（rad/m）：手臂顶在机械停止位上时实测值可能略微越界，
    # 这段时间内不判越限（否则刚上电就被自己的零位漂移急停）。真实越限仍由
    # 物理停止位 + 速度急停兜底。
    static_tolerance: float | list[float] | None = None
    warn_vel_ratio: float | None = None
    estop_vel_ratio: float | None = None
    estop_vel_absolute: float | None = None
    # 全局总开关 + 偏差阈值
    warn_on_pose_deviation: bool | None = None
    warn_pose_threshold: float | None = None

    def __post_init__(self) -> None:
        missing = [name for name in (
            "warn_pos_margin", "estop_pos_margin", "static_tolerance", "warn_vel_ratio",
            "estop_vel_ratio", "estop_vel_absolute", "warn_on_pose_deviation", "warn_pose_threshold",
        ) if getattr(self, name) is None]
        if missing:
            from b601_description.project_config import load_project_config

            cfg = load_project_config()["safety"]
            angle, velocity, pose = cfg["angle"], cfg["velocity"], cfg["pose"]
            defaults = {
                "warn_pos_margin": angle["warn_margin"], "estop_pos_margin": angle["estop_margin"],
                "static_tolerance": angle["static_tolerance"], "warn_vel_ratio": velocity["warn_ratio"],
                "estop_vel_ratio": velocity["estop_ratio"], "estop_vel_absolute": velocity["estop_absolute"],
                "warn_on_pose_deviation": pose["warn_on_deviation"], "warn_pose_threshold": pose["warn_deviation"],
            }
            for name in missing:
                setattr(self, name, defaults[name])
        n = len(self.names)
        self.warn_pos_margin = [float(v) for v in _broadcast(self.warn_pos_margin, n)]
        self.static_tolerance = [float(v) for v in _broadcast(self.static_tolerance, n)]
        self.warn_near_limit = [bool(v) for v in _broadcast(self.warn_near_limit, n)]
        self.warn_pose_deviation_joint = [
            bool(v) for v in _broadcast(self.warn_pose_deviation_joint, n)
        ]

    def index(self, name: str) -> int | None:
        try:
            return self.names.index(name)
        except ValueError:
            return None

    def velocity_estop_threshold(self, i: int) -> float:
        limit = self.estop_vel_ratio * self.vel_max[i]
        if self.estop_vel_absolute > 0.0:
            limit = min(limit, self.estop_vel_absolute)
        return limit

    def velocity_warn_threshold(self, i: int) -> float:
        return self.warn_vel_ratio * self.vel_max[i]


@dataclass
class Evaluation:
    state: str
    reasons: list[str] = field(default_factory=list)
    # 每个关节的明细，供 status 话题的 key-value 用：name -> dict
    joints: dict[str, dict] = field(default_factory=dict)

    @property
    def level(self) -> int:
        return STATE_LEVEL[self.state]

    @property
    def message(self) -> str:
        zh = {NORMAL: "正常", WARNING: "警告", EMERGENCY_STOP: "急停"}[self.state]
        if not self.reasons:
            return zh
        return f"{zh}：" + "；".join(self.reasons)


def evaluate(
    positions: dict[str, float],
    velocities: dict[str, float],
    limits: SafetyLimits,
) -> Evaluation:
    """根据一帧关节状态判定 NORMAL / WARNING / EMERGENCY_STOP。"""
    estop_reasons: list[str] = []
    warn_reasons: list[str] = []
    details: dict[str, dict] = {}

    missing = [n for n in limits.names if n not in positions]

    for i, name in enumerate(limits.names):
        pos = positions.get(name)
        vel = velocities.get(name, 0.0) or 0.0
        details[name] = {
            "position": pos,
            "velocity": vel,
            "min": limits.pos_min[i],
            "max": limits.pos_max[i],
            "vel_max": limits.vel_max[i],
        }
        if pos is None:
            continue

        # ---- 急停条件 ①：角度越限 ----
        # 实际判据 = 限位 + 急停外扩 + 静止零位漂移容差
        slack = limits.estop_pos_margin + limits.static_tolerance[i]
        low_limit = limits.pos_min[i] - slack
        high_limit = limits.pos_max[i] + slack
        over_low = pos < low_limit - EPS
        over_high = pos > high_limit + EPS
        if over_low or over_high:
            details[name]["violation"] = "position"
            estop_reasons.append(
                f"{name} 角度 {pos:+.4f} 越限 [{limits.pos_min[i]:.3f}, {limits.pos_max[i]:.3f}]"
            )

        # ---- 急停条件 ②：角速度越限 ----
        vel_threshold = limits.velocity_estop_threshold(i)
        speed = abs(vel)
        if speed > vel_threshold + EPS:
            details[name]["violation"] = "velocity"
            estop_reasons.append(
                f"{name} 角速度 {speed:.3f} 超过上限 {vel_threshold:.3f}"
            )

        if details[name].get("violation"):
            continue  # 已经是急停，不再叠加软警告理由

        # ---- 警告条件 ①：接近限位（可逐关节关闭）----
        margin = limits.warn_pos_margin[i]
        if limits.warn_near_limit[i] and margin > 0.0:
            if (pos - limits.pos_min[i] < margin) or (limits.pos_max[i] - pos < margin):
                details[name]["warning"] = "near_limit"
                warn_reasons.append(
                    f"{name} 接近限位（{pos:+.3f}，距边界 < {margin:.3f}）"
                )

        # ---- 警告条件 ②：速度偏高 ----
        warn_threshold = limits.velocity_warn_threshold(i)
        if warn_threshold > 0.0 and speed >= warn_threshold:
            details[name]["warning"] = "high_velocity"
            warn_reasons.append(
                f"{name} 角速度 {speed:.3f} 已达上限的 "
                f"{100.0 * speed / limits.vel_max[i]:.0f}%"
            )

        # ---- 警告条件 ③：离开了启动位形（可在关节级与全局级关闭）----
        deviation = abs(pos - limits.startup[i])
        details[name]["deviation"] = deviation
        if (limits.warn_on_pose_deviation
                and limits.warn_pose_deviation_joint[i]
                and deviation > limits.warn_pose_threshold):
            details[name]["warning"] = "off_startup_pose"
            warn_reasons.append(
                f"{name} 偏离启动位形 {deviation:.3f}"
                f"（阈值 {limits.warn_pose_threshold:.2f}）"
            )

    if missing:
        warn_reasons.append("缺少关节反馈：" + ", ".join(missing))

    if estop_reasons:
        return Evaluation(EMERGENCY_STOP, estop_reasons + warn_reasons, details)
    if warn_reasons:
        return Evaluation(WARNING, warn_reasons, details)
    return Evaluation(NORMAL, [], details)
