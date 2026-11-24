#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
case "${1:-}" in
  --execute) args=() ;;
  --dry-run) args=(--dry-run) ;;
  *) echo "用法：$0 --dry-run | --execute"; exit 2 ;;
esac

source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
plan="$(python3 - <<'PY'
from pathlib import Path
import yaml

cfg = yaml.safe_load(Path("ros2_ws/src/b601_description/config/b601_project.yaml").read_text())
plan = cfg["validation"]["pos_vel_five_poses"]
poses = [",".join(f"{name}={value}" for name, value in zip(cfg["robot"]["arm_joints"], target, strict=True))
         for target in plan["targets"]]
print(";".join(poses))
print(plan["slew_rad_s"])
print(plan["sample_sec"])
print(plan["max_delta_rad"])
PY
)"
mapfile -t trial <<< "$plan"
exec ./scripts/dm_pose_steps.py "${args[@]}" --poses "${trial[0]}" --slew "${trial[1]}" \
  --sample-sec "${trial[2]}" --max-delta "${trial[3]}"
