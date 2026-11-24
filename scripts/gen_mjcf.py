#!/usr/bin/env python3
"""gen_mjcf.py —— 从 URDF 生成 MuJoCo 的 MJCF 模型

为什么要自动生成：
    手臂的 kinematics（关节 origin/axis）、惯量、碰撞体都在 URDF 里。手抄进 MJCF 必然走样，
    而且之后改 URDF 就不同步。这里直接从展开后的 URDF 生成，保证"仿真模型 = 真实模型"。

转换规则：
    - fixed 关节：变换**折叠进子 body** 的 pos/quat（MuJoCo 没有 fixed 关节）
    - revolute：hinge 关节，axis 取 URDF 的轴
    - prismatic：slide 关节（夹爪两指），并按 URDF 的 mimic 关系手动同步
    - inertial：mass + pos(质心) + fullinertia(URDF 的惯量张量)
    - collision：直接用 URDF 的 collision 几何（圆柱/盒子），MuJoCo 原生支持
    - 基座固定到 world，加地面与光照

用法：
    python3 scripts/gen_mjcf.py > ros2_ws/src/b601_mujoco/models/b601_arm.xml
"""

from __future__ import annotations

import math
import sys
import xml.etree.ElementTree as ET

import numpy as np


def rpy_to_quat(r, p, y):
    """URDF 的 rpy（R = Rz·Ry·Rx）→ MuJoCo 的 wxyz 四元数"""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y_ = cr * sp * cy + sr * sp * cr * 1.0 - cy * 0  # 展开下方表达式更稳妥
    # 标准公式
    x = sr * cp * cy - cr * sp * sy
    y_ = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, y_, z


def rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def mat_to_quat(R):
    """旋转矩阵 → wxyz 四元数"""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return w, x, y, z


class Link:
    def __init__(self, el):
        self.name = el.get("name")
        self.mass = 0.0
        self.com = np.zeros(3)
        self.inertia = None       # (ixx, iyy, izz, ixy, ixz, iyz)
        self.geoms = []           # (kind, params, origin_xyz, origin_rpy)
        inertial = el.find("inertial")
        if inertial is not None:
            m = inertial.find("mass")
            self.mass = float(m.get("value")) if m is not None else 0.0
            org = inertial.find("origin")
            if org is not None and org.get("xyz"):
                self.com = np.array([float(v) for v in org.get("xyz").split()])
            it = inertial.find("inertia")
            if it is not None:
                self.inertia = (float(it.get("ixx")), float(it.get("iyy")), float(it.get("izz")),
                                float(it.get("ixy")), float(it.get("ixz")), float(it.get("iyz")))
        for col in el.findall("collision"):
            org = col.find("origin")
            xyz = np.zeros(3)
            rpy = np.zeros(3)
            if org is not None:
                if org.get("xyz"):
                    xyz = np.array([float(v) for v in org.get("xyz").split()])
                if org.get("rpy"):
                    rpy = np.array([float(v) for v in org.get("rpy").split()])
            geom = col.find("geometry")
            if geom.find("cylinder") is not None:
                c = geom.find("cylinder")
                self.geoms.append(("cylinder", (float(c.get("radius")),), xyz, rpy,
                                   float(c.get("length"))))
            elif geom.find("box") is not None:
                b = geom.find("box")
                self.geoms.append(("box", tuple(float(v) for v in b.get("size").split()), xyz, rpy, None))


class JointEl:
    def __init__(self, el):
        self.name = el.get("name")
        self.type = el.get("type")
        self.parent = el.find("parent").get("link")
        self.child = el.find("child").get("link")
        org = el.find("origin")
        self.xyz = np.zeros(3)
        self.rpy = np.zeros(3)
        if org is not None:
            if org.get("xyz"):
                self.xyz = np.array([float(v) for v in org.get("xyz").split()])
            if org.get("rpy"):
                self.rpy = np.array([float(v) for v in org.get("rpy").split()])
        ax = el.find("axis")
        self.axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else np.zeros(3)
        lim = el.find("limit")
        self.limit = None
        if lim is not None:
            self.limit = (float(lim.get("lower", "0")), float(lim.get("upper", "0")))
        mimic = el.find("mimic")
        self.mimic = mimic.get("joint") if mimic is not None else None
        self.mimic_mult = float(mimic.get("multiplier", "1")) if mimic is not None else 1.0


def main() -> int:
    sys.path.insert(0, "ros2_ws/src/b601_gravity_id")
    from b601_gravity_id.model import expand_xacro

    root = ET.fromstring(expand_xacro(hardware="mock"))
    links = {el.get("name"): Link(el) for el in root.findall("link")}
    joints = [JointEl(el) for el in root.findall("joint")]
    by_child = {j.child: j for j in joints}
    children_of = {}
    for j in joints:
        children_of.setdefault(j.parent, []).append(j)

    out = []
    w = out.append
    w('<!-- 本文件由 scripts/gen_mjcf.py 从 URDF 自动生成，请勿手改 -->')
    w('<mujoco model="b601_dm">')
    w('  <compiler angle="radian" autolimits="true"/>')
    w('  <!-- 1 ms 步长、implicitfast 积分器：机械臂这种刚性系统稳定且快 -->')
    w('  <option timestep="0.001" integrator="implicitfast" iterations="50"/>')
    w('  <visual>')
    w('    <global offwidth="1280" offheight="720"/>')
    w('    <headlight ambient="0.35 0.35 0.35" diffuse="0.7 0.7 0.7" specular="0.1 0.1 0.1"/>')
    w('  </visual>')
    w('  <asset>')
    w('    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.25 0.4 0.55" '
      'rgb2="0.05 0.06 0.09" width="512" height="3072"/>')
    w('    <texture name="grid" type="2d" builtin="checker" mark="edge" rgb1="0.22 0.28 0.31" '
      'rgb2="0.10 0.14 0.16" markrgb="0.72 0.78 0.80" width="300" height="300"/>')
    w('    <material name="ground" texture="grid" texuniform="true" texrepeat="6 6" reflectance="0.1"/>')
    w('    <material name="seeed_yellow" rgba="0.73 0.84 0.12 1"/>')
    w('    <material name="dark_grey" rgba="0.24 0.27 0.27 1"/>')
    w('    <material name="silver" rgba="0.83 0.84 0.83 1"/>')
    w('    <material name="matte_black" rgba="0.07 0.07 0.07 1"/>')
    w('    <material name="accent_blue" rgba="0.10 0.45 0.85 1"/>')
    w('  </asset>')
    w('  <worldbody>')
    w('    <light pos="0.6 -0.4 1.6" dir="-0.4 0.3 -1" directional="true"/>')
    w('    <geom name="floor" type="plane" size="1.5 1.5 0.05" material="ground" '
      'friction="1.0 0.01 0.001"/>')
    w('    <camera name="overview" pos="0.75 -0.75 0.75" xyaxes="1 1 0 -0.4 0.4 1" fovy="45"/>')

    color = {"base_link": "dark_grey", "link1": "seeed_yellow", "link2": "seeed_yellow",
             "link3": "seeed_yellow", "link4": "silver", "link5": "silver",
             "link6": "matte_black", "end_link": "accent_blue",
             "finger_left_link": "matte_black", "finger_right_link": "matte_black"}

    def emit_body(link_name: str, indent: int, extra_xyz=None, extra_rpy=None, depth=0):
        """递归输出 body；fixed 关节的变换折叠进子 body 的 pos/quat"""
        pad = "  " * indent
        link = links[link_name]
        j = by_child.get(link_name)
        pos = np.zeros(3) if extra_xyz is None else np.array(extra_xyz, float)
        R = np.eye(3) if extra_rpy is None else rpy_to_matrix(extra_rpy)
        joint_line = None
        if j is not None and j.type != "fixed":
            # 该 link 由一个可动关节驱动：body 的位置就是关节 origin
            pos = j.xyz.copy()
            R = rpy_to_matrix(j.rpy)
            axis = j.axis / np.linalg.norm(j.axis)
            jt = "hinge" if j.type in ("revolute", "continuous") else "slide"
            lim = f' range="{j.limit[0]:.5f} {j.limit[1]:.5f}"' if j.limit else ""
            mimic = ""
            if j.mimic:
                mimic = f' <!-- mimic {j.mimic} x{j.mimic_mult} -->'
            joint_line = (f'{pad}  <joint name="{j.name}" type="{jt}" '
                          f'axis="{axis[0]:.6f} {axis[1]:.6f} {axis[2]:.6f}"{lim} '
                          f'damping="0.02" armature="0.01"/>{mimic}')

        q = mat_to_quat(R)
        w(f'{pad}<body name="{link_name}" pos="{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}" '
          f'quat="{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}">')
        if link.inertia is not None and link.mass > 0:
            ixx, iyy, izz, ixy, ixz, iyz = link.inertia
            w(f'{pad}  <inertial pos="{link.com[0]:.6f} {link.com[1]:.6f} {link.com[2]:.6f}" '
              f'mass="{link.mass:.6f}" fullinertia="{ixx:.8f} {iyy:.8f} {izz:.8f} '
              f'{ixy:.8f} {ixz:.8f} {iyz:.8f}"/>')
        if joint_line:
            w(joint_line)
        for kind, pars, xyz, rpy, length in link.geoms:
            rq = mat_to_quat(rpy_to_matrix(rpy))
            mat = color.get(link_name, "silver")
            if kind == "cylinder":
                # MuJoCo 的 cylinder 沿 z 轴、half-height 语义
                w(f'{pad}  <geom name="{link_name}_geom" type="cylinder" '
                  f'size="{pars[0]:.6f} {length / 2:.6f}" '
                  f'pos="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}" '
                  f'quat="{rq[0]:.6f} {rq[1]:.6f} {rq[2]:.6f} {rq[3]:.6f}" '
                  f'material="{mat}" contype="0" conaffinity="0"/>')
            else:
                w(f'{pad}  <geom name="{link_name}_geom" type="box" '
                  f'size="{pars[0] / 2:.6f} {pars[1] / 2:.6f} {pars[2] / 2:.6f}" '
                  f'pos="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}" '
                  f'quat="{rq[0]:.6f} {rq[1]:.6f} {rq[2]:.6f} {rq[3]:.6f}" '
                  f'material="{mat}" contype="0" conaffinity="0"/>')

        # 子 link
        for cj in children_of.get(link_name, []):
            if cj.type == "fixed":
                # MuJoCo 的 body pos/quat 就是「父 link 系 → 本 link 系」的变换，
                # 等于 URDF 里该关节的 origin —— 不需要任何叠加。
                emit_body_fixed(cj.child, indent + 1, cj.xyz, rpy_to_matrix(cj.rpy), depth + 1)
            else:
                emit_body(cj.child, indent + 1, depth=depth + 1)
        w(f'{pad}</body>')

    def emit_body_fixed(link_name, indent, pos, R, depth=0):
        """fixed link：直接给定 world→body 的 pos/R（相对父 body 的累积结果）"""
        pad = "  " * indent
        link = links[link_name]
        q = mat_to_quat(R)
        w(f'{pad}<body name="{link_name}" pos="{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}" '
          f'quat="{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}">')
        if link.inertia is not None and link.mass > 0:
            ixx, iyy, izz, ixy, ixz, iyz = link.inertia
            w(f'{pad}  <inertial pos="{link.com[0]:.6f} {link.com[1]:.6f} {link.com[2]:.6f}" '
              f'mass="{link.mass:.6f}" fullinertia="{ixx:.8f} {iyy:.8f} {izz:.8f} '
              f'{ixy:.8f} {ixz:.8f} {iyz:.8f}"/>')
        for kind, pars, xyz, rpy, length in link.geoms:
            rq = mat_to_quat(rpy_to_matrix(rpy))
            mat = color.get(link_name, "silver")
            if kind == "cylinder":
                w(f'{pad}  <geom name="{link_name}_geom" type="cylinder" '
                  f'size="{pars[0]:.6f} {length / 2:.6f}" '
                  f'pos="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}" '
                  f'quat="{rq[0]:.6f} {rq[1]:.6f} {rq[2]:.6f} {rq[3]:.6f}" '
                  f'material="{mat}" contype="0" conaffinity="0"/>')
            else:
                w(f'{pad}  <geom name="{link_name}_geom" type="box" '
                  f'size="{pars[0] / 2:.6f} {pars[1] / 2:.6f} {pars[2] / 2:.6f}" '
                  f'pos="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}" '
                  f'quat="{rq[0]:.6f} {rq[1]:.6f} {rq[2]:.6f} {rq[3]:.6f}" '
                  f'material="{mat}" contype="0" conaffinity="0"/>')
        for cj in children_of.get(link_name, []):
            if cj.type == "fixed":
                emit_body_fixed(cj.child, indent + 1, cj.xyz, rpy_to_matrix(cj.rpy), depth + 1)
            else:
                emit_body(cj.child, indent + 1, depth=depth + 1)
        w(f'{pad}</body>')

    # 根：world 下挂 base_link（world→base_link 是 fixed）
    base_j = by_child.get("base_link")
    if base_j is not None and base_j.type == "fixed":
        emit_body_fixed("base_link", 2, base_j.xyz.copy(), rpy_to_matrix(base_j.rpy))
    else:
        emit_body("base_link", 2)

    w('  </worldbody>')
    w('')
    w('  <!-- 执行器：给六个关节各一个力矩执行器（力矩控制 = 我们的 MIT 控制律）-->')
    w('  <actuator>')
    for jn in ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]:
        j = by_id = next(x for x in joints if x.name == jn)
        lim = abs(j.limit[1] - j.limit[0]) if j.limit else 100
        eff = (27.0 if jn in ("joint1", "joint2", "joint3") else 7.0)
        w(f'    <motor name="{jn}_motor" joint="{jn}" gear="1" '
          f'ctrlrange="{-eff:.1f} {eff:.1f}" ctrllimited="true"/>')
    w('  </actuator>')
    w('')
    w('  <!-- 传感器：便于和真机 /joint_states 对照 -->')
    w('  <sensor>')
    for jn in ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]:
        w(f'    <jointpos name="{jn}_pos" joint="{jn}"/>')
        w(f'    <jointvel name="{jn}_vel" joint="{jn}"/>')
        w(f'    <jointactuatorfrc name="{jn}_frc" joint="{jn}"/>')
    w('  </sensor>')
    w('</mujoco>')
    print("\n".join(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
