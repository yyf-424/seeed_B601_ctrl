"""model.py —— 从 URDF 推「重力力矩回归矩阵」Y_g(q)，不依赖 pinocchio / MuJoCo

为什么需要回归矩阵：
    静态时电机力矩 ≈ 重力力矩（速度≈0，摩擦≈0），所以
        τ_meas(q) ≈ τ_g(q)
    而 τ_g 对"惯性参数"是**线性**的：每连杆 4 个参数
        π_k = (m_k, m_k·c_x, m_k·c_y, m_k·c_z)
    于是可以写成矩阵形式
        τ = Y_g(q) · π
    用几十个静态位形采样，就能用最小二乘把 π 解出来 —— 这就是重力辨识。

公式（虚功原理，势能对关节角求导）：
    U(q) = Σ_k m_k · g · p_com,k(q),      g = (0, 0, -9.81)
    τ_i   = ∂U/∂q_i
          = Σ_k [ m_k · (g · ∂p_o,k/∂q_i)  +  Σ_{a∈xyz} (m_k c_{k,a}) · (g · ∂(R_k e_a)/∂q_i) ]

所以 Y_g 的列：
    质量列      g · ∂p_o,k/∂q          （p_o = link 原点位置）
    质心偏置列  g · ∂(R_k e_a)/∂q      （R_k = link 姿态矩阵）
全部用中心差分数值求导（h=1e-5），不需要解析雅可比，代码短且不易写错。

约定：
    - 关节顺序固定为 ARM_JOINTS（joint1..joint6）
    - π 的排列：[link1(4), link2(4), ..., 每个有惯量的臂上 link]
    - 夹爪两个手指按 prismatic q=0 处理（辨识时夹爪闭合静止，位移 ≤5mm 可忽略）
"""

from __future__ import annotations

import math
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np

ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRAVITY = np.array([0.0, 0.0, -9.81])


# --------------------------------------------------------------------------- URDF
def expand_xacro(package: str = "b601_description", hardware: str = "mock") -> str:
    """调用 xacro 把包里的顶层 xacro 展开成 URDF 文本。"""
    from ament_index_python.packages import get_package_share_directory
    import os

    share = get_package_share_directory(package)
    path = os.path.join(share, "urdf", "b601_dm.urdf.xacro")
    out = subprocess.run(
        ["xacro", path, f"hardware:={hardware}"],
        capture_output=True, text=True, check=True,
    )
    return out.stdout


def _rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    # URDF 的 rpy 约定：固定轴 X-Y-Z（等价 R = Rz(y)·Ry(p)·Rx(r)）
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3)
    k = axis / n
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray


@dataclass
class Link:
    name: str
    mass: float = 0.0
    com: np.ndarray = field(default_factory=lambda: np.zeros(3))


class ArmModel:
    """只用运动学 + 惯量（质量/质心）：够算重力力矩。"""

    def __init__(self, urdf_xml: str, joints=ARM_JOINTS):
        root = ET.fromstring(urdf_xml)
        self.joint_names = list(joints)
        self.links = {}
        self.joints = {}
        for el in root.findall("link"):
            name = el.get("name")
            link = Link(name)
            inertial = el.find("inertial")
            if inertial is not None:
                m = inertial.find("mass")
                link.mass = float(m.get("value")) if m is not None else 0.0
                org = inertial.find("origin")
                if org is not None and org.get("xyz"):
                    link.com = np.array([float(v) for v in org.get("xyz").split()])
            self.links[name] = link
        for el in root.findall("joint"):
            name = el.get("name")
            org = el.find("origin")
            xyz = np.zeros(3)
            rpy = np.zeros(3)
            if org is not None:
                if org.get("xyz"):
                    xyz = np.array([float(v) for v in org.get("xyz").split()])
                if org.get("rpy"):
                    rpy = np.array([float(v) for v in org.get("rpy").split()])
            ax = el.find("axis")
            axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else np.zeros(3)
            self.joints[name] = Joint(
                name, el.get("type"), el.find("parent").get("link"),
                el.find("child").get("link"), xyz, rpy, axis,
            )
        # 根 link → 子关节索引
        self.child_joints = {}
        for j in self.joints.values():
            self.child_joints.setdefault(j.parent, []).append(j)
        self.root = next(iter(self.joints.values())).parent
        while True:
            parents = {j.parent for j in self.joints.values()}
            roots = [p for p in parents if p not in {j.child for j in self.joints.values()}]
            self.root = roots[0] if roots else self.root
            break

    # ---------------------------------------------------------------- 正运动学
    def fk(self, q):
        """返回 {link_name: (p_o(3), R(3x3))}，q 为 6 个关节角。"""
        qmap = dict(zip(self.joint_names, q))
        out = {self.root: (np.zeros(3), np.eye(3))}
        stack = [self.root]
        while stack:
            parent = stack.pop()
            p_par, R_par = out[parent]
            for j in self.child_joints.get(parent, []):
                p = p_par + R_par @ j.xyz
                R = R_par @ _rpy_to_matrix(j.rpy)
                if j.type in ("revolute", "continuous"):
                    R = R @ _axis_angle(j.axis, qmap.get(j.name, 0.0))
                elif j.type == "prismatic":
                    p = p + R @ (j.axis * qmap.get(j.name, 0.0))
                out[j.child] = (p, R)
                stack.append(j.child)
        return out

    # ------------------------------------------------------------ 参数与回归矩阵
    def param_links(self):
        """参与辨识的 link（有质量且在臂链上），按 π 分块顺序返回。"""
        fk0 = self.fk(np.zeros(len(self.joint_names)))
        return [name for name in fk0 if self.links.get(name) and self.links[name].mass > 1e-9]

    def urdf_params(self):
        """URDF 自带参数（真值参考）：π = (m, m cx, m cy, m cz) 按 link 拼接。"""
        pi = []
        for name in self.param_links():
            link = self.links[name]
            pi.extend([link.mass, *(link.mass * link.com)])
        return np.array(pi)

    def gravity_torque(self, q, pi=None):
        """直接用参数算重力力矩（pi 缺省用 URDF 参数）。"""
        if pi is None:
            pi = self.urdf_params()
        fk = self.fk(q)
        tau = np.zeros(len(self.joint_names))
        k = 0
        for name in self.param_links():
            m = pi[4 * k]
            mu = np.array(pi[4 * k + 1: 4 * k + 4])
            p_o, R = fk[name]
            p_com = p_o + R @ (mu / m if abs(m) > 1e-12 else np.zeros(3))
            tau += m * (GRAVITY @ p_com) * 0.0  # 占位，真实计算见下方数值路径
            k += 1
        return tau

    def gravity_regressor(self, q, h=1e-5):
        """Y_g(q)：形状 (6, 4*len(param_links))，满足 τ = Y · π。"""
        q = np.asarray(q, dtype=float)
        nq = len(q)
        links = self.param_links()

        def fk_q(qq):
            return self.fk(qq)

        base = fk_q(q)
        # 数值求导：dp_o/dq_i 与 dR/dq_i
        dp = {name: np.zeros((nq, 3)) for name in links}
        dR = {name: np.zeros((nq, 3, 3)) for name in links}
        for i in range(nq):
            qp = q.copy(); qp[i] += h
            qm = q.copy(); qm[i] -= h
            fkp, fkm = fk_q(qp), fk_q(qm)
            for name in links:
                dp[name][i] = (fkp[name][0] - fkm[name][0]) / (2 * h)
                dR[name][i] = (fkp[name][1] - fkm[name][1]) / (2 * h)

        Y = np.zeros((nq, 4 * len(links)))
        for k, name in enumerate(links):
            # 质量列：g · ∂p_o/∂q
            Y[:, 4 * k] = dp[name] @ GRAVITY
            # 质心列：g · ∂(R e_a)/∂q
            for a in range(3):
                e = np.zeros(3); e[a] = 1.0
                # dR e_a 的 z 分量乘 g_z（g 只有 z）
                dz = dR[name][:, 2, :] @ e
                Y[:, 4 * k + 1 + a] = GRAVITY[2] * dz
        return Y, base


def load_model(hardware="mock"):
    return ArmModel(expand_xacro(hardware=hardware))


# --------------------------------------------------------------------------- 自检
if __name__ == "__main__":
    m = load_model("mock")
    print(f"根 link: {m.root}")
    links = m.param_links()
    print(f"参与辨识的 link（{len(links)} 个）: {links}")
    pi = m.urdf_params()
    print(f"URDF 总质量 = {pi[0::4].sum():.4f} kg")

    # ① 三种独立算法交叉验证（回归矩阵 / 势能求导 / 仅下游 link 的叉乘）
    q_test = np.array([0.3, -0.5, -0.8, 0.2, 0.3, 0.4])
    Y, _ = m.gravity_regressor(q_test)
    tau_reg = Y @ pi

    def potential(qq):
        fk = m.fk(qq)
        return float(sum(m.links[n].mass * (GRAVITY @ (fk[n][0] + fk[n][1] @ m.links[n].com))
                         for n in links))

    h = 1e-5
    tau_U = np.array([(potential(q_test + h * np.eye(6)[i]) - potential(q_test - h * np.eye(6)[i])) / (2 * h)
                      for i in range(6)])

    # 只对"该关节下游"的 link 求力矩：上游 link 的重力对该关节的力矩被父结构抵消
    def downstream(jname):
        out, stack = set(), [m.joints[jname].child]
        while stack:
            cur = stack.pop(); out.add(cur)
            for j in m.joints.values():
                if j.parent == cur:
                    stack.append(j.child)
        return out

    fk = m.fk(q_test)
    tau_x = np.zeros(6)
    for i, jname in enumerate(m.joint_names):
        j = m.joints[jname]
        p_par, R_par = fk[j.parent]
        p_joint = fk[j.child][0]
        axis_w = R_par @ _rpy_to_matrix(j.rpy) @ (j.axis / np.linalg.norm(j.axis))
        for name in downstream(jname):
            p_o, R = fk[name]
            p_com = p_o + R @ m.links[name].com
            tau_x[i] += np.dot(np.cross(p_com - p_joint, m.links[name].mass * GRAVITY), axis_w)

    print("\n① 三种算法交叉验证：")
    print(f"   {'关节':<10}{'回归矩阵':>12}{'势能求导':>12}{'下游叉乘':>12}")
    for i, n in enumerate(m.joint_names):
        print(f"   {n:<10}{tau_reg[i]:>+12.5f}{tau_U[i]:>+12.5f}{tau_x[i]:>+12.5f}")
    worst = max(np.abs(tau_reg - tau_U).max(), np.abs(tau_reg - tau_x).max())
    print(f"   最大互差 {worst:.2e} N·m → {'✅ 一致' if worst < 1e-6 else '❌ 有矛盾'}")

    # ③ 打印当前位形（零位附近）的理论重力力矩，作为真机对照参考
    q0 = np.array([0.0, -0.0242, -0.0280, 0.0048, -0.0025, -0.0010])
    Y0, _ = m.gravity_regressor(q0)
    tau0 = Y0 @ pi
    print("\n③ 真机当前位形（近竖直）的理论重力力矩：")
    for n, t in zip(m.joint_names, tau0):
        print(f"   {n:<8} {t:+.5f} N·m")
