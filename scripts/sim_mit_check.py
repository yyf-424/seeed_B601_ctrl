#!/usr/bin/env python3
"""用静态位形对比 MuJoCo 重力力矩和当前实机辨识模型。"""

import argparse
from pathlib import Path
import sys

import numpy as np

from project_config import ROOT, load_project_config


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml", type=Path, default=ROOT / "ros2_ws/src/b601_mujoco/models/b601_arm.xml")
    args = parser.parse_args()

    import mujoco

    sys.path.insert(0, str(ROOT / "ros2_ws/src/b601_gravity_id"))
    from b601_gravity_id.model import load_model

    project = load_project_config()
    arm = load_model("mujoco")
    links = [name for name in arm.param_links() if name not in ("base_link", "link1")]
    arm.param_links = lambda: links
    gravity_params = project["controllers"]["mit"]["shared"]["gravity_params"]
    model = mujoco.MjModel.from_xml_path(str(args.xml))
    data = mujoco.MjData(model)
    joints = project["robot"]["arm_joints"]
    pose = project["tools"]["sim_mit_check"]["start_rad"]

    for name, position in zip(joints, pose):
        data.joint(name).qpos[0] = position
    mujoco.mj_forward(model, data)
    reference = np.array([float(data.joint(name).qfrc_bias[0]) for name in joints])
    regressor, _ = arm.gravity_regressor(pose)
    predicted = -(regressor @ gravity_params)
    if not np.all(np.isfinite([reference, predicted])) or not np.any(reference) or not np.any(predicted):
        print("静态位形未得到有限的重力力矩")
        return 1
    print("MuJoCo 重力力矩 N·m", np.round(reference, 4))
    print("实机辨识模型 N·m", np.round(predicted, 4))
    print(f"重力模型与 MuJoCo 最大静态差值 {np.max(np.abs(predicted - reference)):.4f} N·m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
