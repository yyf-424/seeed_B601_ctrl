#!/usr/bin/env python3
"""sim_mit_check.py —— 在 MuJoCo 里验证 MIT 力控律（重力前馈 + 摩擦 + 增益调度）

为什么先用仿真：
    真机上出过一次事故（机械臂抽风、腕部转了 175°）。在仿真里可以把每种组合都跑一遍，
    用 MuJoCo 的物理真值判断"符号对不对、增益合不合适"，而不是靠现场试。

验证的四件事：
    ① 重力前馈到底该用哪个符号：分别用 +g 和 −g 两个方向做前馈，看哪个能稳住
    ② 有/无重力前馈的稳态下垂差多少（kp 相同时）
    ③ 小刚度悬浮（kp_hold）下，外力推动后是否"停在新位置"（柔顺、无积分项）
    ④ 收到目标后增益调度（kp_move）能否快速到位

用法：
    python3 scripts/sim_mit_check.py                     # 完整四组
    python3 scripts/sim_mit_check.py --gravity fitted    # 用真机辨识出的参数做前馈
"""

from __future__ import annotations

import argparse
import re
import sys

import numpy as np

from project_config import load_project_config

PROJECT = load_project_config()
SETTINGS = PROJECT["tools"]["sim_mit_check"]


def load_fitted_params(path=".verify/gravity_params.yaml"):
    """读取 identify.py 生成的参数（8 个 link × 4，顺序 = kGravityLinks）"""
    txt = open(path, encoding="utf-8").read()
    vals = []
    for m in re.finditer(r"\[\s*([-+0-9.eE]+),\s*([-+0-9.eE]+),\s*([-+0-9.eE]+),\s*([-+0-9.eE]+)\s*\]",
                         txt.split("link_params:")[1]):
        vals.extend(float(m.group(i)) for i in (1, 2, 3, 4))
    return np.array(vals)


# 按等效惯量 I = [0.111, 0.414, 0.297, 0.047, 0.021, 0.0106] kg·m² 算的临界阻尼
# kd = 2√(kp·I)。用统一 kd 会让最轻的 joint6 离散发散（kd·dt/I > 2）。
KD_HOLD = SETTINGS["kd_hold"]     # kp=6
KD_MOVE = SETTINGS["kd_move"]     # kp=60


class Sim:
    def __init__(self, xml, gravity_source="truth", kp_hold=SETTINGS["kp_hold"], kd_hold=None,
                 kp_move=SETTINGS["kp_move"], kd_move=None,
                 move_thr=SETTINGS["move_threshold_rad"], hold_thr=SETTINGS["hold_threshold_rad"],
                 friction_scale=SETTINGS["friction_scale"],
                 sub_steps=SETTINGS["substeps"]):
        import mujoco
        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.data = mujoco.MjData(self.model)
        self.qadr = [self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"joint{i}")] for i in range(1, 7)]
        self.vadr = [self.model.jnt_dofadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"joint{i}")] for i in range(1, 7)]
        self.gravity_source = gravity_source
        self.kp_hold = kp_hold
        self.kd_hold = np.array(kd_hold if kd_hold is not None else KD_HOLD, float)
        self.kp_move = kp_move
        self.kd_move = np.array(kd_move if kd_move is not None else KD_MOVE, float)
        self.move_thr, self.hold_thr = move_thr, hold_thr
        self.friction_scale = friction_scale
        self.sub_steps = sub_steps
        self.kp = np.full(6, kp_hold)
        self.kd = np.array(kd_hold if kd_hold is not None else KD_HOLD, float)
        self.mode_move = False
        self.q_des = None
        self.ext_force = 0.0     # 施加到末端的外力（N），用于"推一下"测试
        self.q_handle = None
        self._load_gravity_model()

    def _load_gravity_model(self):
        """离线模型前馈：用 b601_gravity_id 的 python 实现 + 参数"""
        self.py_model = None
        self.pi = None
        if self.gravity_source == "zero":
            return np.zeros(6)
        if self.gravity_source in ("urdf", "fitted"):
            sys.path.insert(0, "ros2_ws/src/b601_gravity_id")
            from b601_gravity_id.model import load_model
            self.py_model = load_model("dm")
            all_links = self.py_model.param_links()
            links = [n for n in all_links if n not in ("base_link", "link1")]
            self.py_model.param_links = lambda: links
            if self.gravity_source == "urdf":
                pi_all = self.py_model.urdf_params()     # 注意：param_links 已替换，顺序变了
                self.pi = np.array([pi_all[4 * i:4 * i + 4].tolist() for i in range(len(links))]).ravel()
            else:
                # 从真机辨识结果读取（identify.py 生成）
                self.pi = load_fitted_params()

    # ------------------------------------------------------------------ 基础
    def q(self):
        return np.array([self.data.qpos[a] for a in self.qadr])

    def qd(self):
        return np.array([self.data.qvel[a] for a in self.vadr])

    def gravity_torque(self):
        """返回「关节系下需要施加的保持力矩」——符号以 +q 方向为正"""
        if self.gravity_source == "truth":
            self.mj.mj_forward(self.model, self.data)
            return self.data.qfrc_bias[:6].copy()
        if self.gravity_source == "inverted":
            self.mj.mj_forward(self.model, self.data)
            return -self.data.qfrc_bias[:6].copy()
        if self.gravity_source == "zero":
            return np.zeros(6)
        if self.gravity_source in ("urdf", "fitted"):
            Y, _ = self.py_model.gravity_regressor(self.q())
            return -(Y @ self.pi)          # 模型输出是 −g，取反得到保持力矩
        raise ValueError(self.gravity_source)

    def reset(self, q0):
        self.mj.mj_resetData(self.model, self.data)
        for a, v in zip(self.qadr, q0):
            self.data.qpos[a] = v
        self.mj.mj_forward(self.model, self.data)
        self.q_des = np.array(q0, float)
        self.kp = np.full(6, self.kp_hold)
        self.mode_move = False

    def step(self, ext_force=0.0):
        mj, d = self.mj, self.data
        dt = self.model.opt.timestep * self.sub_steps
        q, qd = self.q(), self.qd()
        # 增益调度（与时序一致：滞回 + 一阶平滑）
        err = self.q_des - q
        if np.abs(err).max() > self.move_thr:
            self.mode_move = True
        elif np.abs(err).max() < self.hold_thr and np.abs(qd).max() < 0.30:
            self.mode_move = False
        kp_t = np.full(6, self.kp_move if self.mode_move else self.kp_hold)
        kd_t = self.kd_move if self.mode_move else self.kd_hold
        self.kp += (kp_t - self.kp) * min(1.0, dt / 0.35)
        self.kd += (kd_t - self.kd) * min(1.0, dt / 0.35)
        kd = self.kd
        tau = self.kp * err + kd * (-qd) + self.gravity_torque()
        for _ in range(self.sub_steps):
            d.ctrl[:6] = tau
            if ext_force:
                # 在末端 link 上施加沿 +x 的外力，模拟"用手推"
                bid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "end_link")
                d.xfrc_applied[bid, 0] = ext_force
            for i in range(6):
                d.ctrl[i] = tau[i]      # 力矩执行器（gear=1）
            mj.mj_step(self.model, self.data)
            d.xfrc_applied[:] = 0.0

    def run(self, seconds, ext_force=0.0, log=None):
        n = int(round(seconds / (self.model.opt.timestep * self.sub_steps)))
        for k in range(n):
            self.step(ext_force if callable(ext_force) else ext_force)
            if log is not None and k % SETTINGS["snapshot_divider"] == 0:
                log.append((self.q(), self.q_des.copy(), self.kp.copy()))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", default="ros2_ws/src/b601_mujoco/models/b601_arm.xml")
    ap.add_argument("--gravity", default="truth",
                    choices=["truth", "inverted", "urdf", "fitted"])
    ap.add_argument("--friction-scale", type=float, default=SETTINGS["friction_scale"])
    args = ap.parse_args()

    # 起始位形：手臂前伸（仿真里不会像真机那样顶在限位上）
    q0 = np.array(SETTINGS["start_rad"])
    print(f"重力前馈来源 = {args.gravity}；起始位形 q = {np.array2string(q0, precision=3)}")

    # ---------- ① 前馈符号：稳/不稳 ----------
    print("\n① 前馈符号检验（kp=40，观察 3 秒后是否还在起始位形附近）")
    for src in ("truth", "inverted"):
        sim = Sim(args.xml, gravity_source=src, kp_hold=SETTINGS["sign_test_kp"])
        sim.reset(q0)
        sim.run(SETTINGS["observe_sec"])
        dq = sim.q() - q0
        print(f"   前馈={src:<9} 3s 后偏差 = {np.array2string(dq, precision=4)}  "
              f"最大 {np.abs(dq).max():.4f} rad → "
              + ("✅ 稳住" if np.abs(dq).max() < SETTINGS['sign_tol_rad'] else "❌ 发散/下垂"))

    # ---------- ② 有/无前馈的下垂对比 ----------
    print("\n② 有无重力前馈的稳态下垂（kp=40，等 3 秒）")
    sim = Sim(args.xml, gravity_source="zero", kp_hold=SETTINGS["sign_test_kp"])  # 纯 PD（无前馈）
    sim.reset(q0)
    sim.run(SETTINGS["observe_sec"])
    droop_pd = sim.q() - q0
    sim2 = Sim(args.xml, gravity_source="truth", kp_hold=SETTINGS["sign_test_kp"])
    sim2.reset(q0)
    sim2.run(SETTINGS["observe_sec"])
    droop_ff = sim2.q() - q0
    print(f"   纯 PD（无前馈）: {np.array2string(droop_pd, precision=4)}  最大 {np.abs(droop_pd).max():.4f} rad")
    print(f"   带重力前馈     : {np.array2string(droop_ff, precision=4)}  最大 {np.abs(droop_ff).max():.4f} rad")

    # ---------- ③ 小刚度悬浮 + 外力推动 ----------
    print("\n③ 小刚度悬浮（kp_hold=6）+ 推一下")
    sim = Sim(args.xml, gravity_source=args.gravity, kp_hold=SETTINGS["kp_hold"])
    sim.reset(q0)
    sim.run(SETTINGS["push_rest_sec"])
    q_before = sim.q()
    sim.run(SETTINGS["push_sec"], ext_force=SETTINGS["push_force_n"])          # 3 N 推力 0.8 秒
    q_pushed = sim.q()
    sim.run(SETTINGS["push_rest_sec"])                          # 松手后
    q_after = sim.q()
    print(f"   推之前      : {np.array2string(q_before, precision=4)}")
    print(f"   推 0.8s 后  : {np.array2string(q_pushed, precision=4)}  "
          f"被推动 {np.abs(q_pushed - q_before).max():.4f} rad")
    print(f"   松手 2s 后  : {np.array2string(q_after, precision=4)}  "
          f"回到原位差 {np.abs(q_after - q_before).max():.4f} rad")
    print("   （没有积分项，所以松手后停在目标附近而不是弹回 —— 这就是「柔顺」）")

    # ---------- ④ 目标阶跃：增益调度 ----------
    print("\n④ 收到目标后的增益调度（kp 6 → 60）")
    sim = Sim(args.xml, gravity_source=args.gravity, kp_hold=SETTINGS["kp_hold"])
    sim.reset(q0)
    sim.run(SETTINGS["initial_hold_sec"])
    target = q0 + np.array(SETTINGS["step_rad"])
    sim.q_des = target.copy()
    log = []
    t = 0.0
    dt_ctrl = sim.model.opt.timestep * sim.sub_steps
    reached = None
    while t < SETTINGS["observe_sec"]:
        sim.step()
        t += dt_ctrl
        err = np.abs(sim.q() - target).max()
        if reached is None and err < SETTINGS["step_tol_rad"]:
            reached = t
    print(f"   目标增量 {np.array2string(target - q0, precision=3)}")
    print(f"   到达（误差<0.02 rad）用时：{reached if reached else '>3s'}")
    print(f"   最终误差 {np.abs(sim.q() - target).max():.4f} rad，"
          f"kp 终值 {sim.kp[0]:.1f}")

    # ---------- ⑤ 前馈来源对比：真值 / URDF(CAD) / 真机辨识 ----------
    print("\n⑤ 三种前馈来源的稳态下垂（kp_hold=6，等 3 秒）")
    print("   truth = MuJoCo 物理真值；urdf = CAD 惯量；fitted = 真机静态辨识参数")
    for src in ("truth", "urdf", "fitted"):
        try:
            sim = Sim(args.xml, gravity_source=src, kp_hold=SETTINGS["kp_hold"])
            sim.reset(q0)
            sim.run(SETTINGS["observe_sec"])
            dq = sim.q() - q0
            print(f"   {src:<8} 下垂 {np.array2string(dq, precision=4)}  "
                  f"最大 {np.abs(dq).max():.4f} rad")
        except FileNotFoundError:
            print(f"   {src:<8} 跳过（缺参数文件）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
