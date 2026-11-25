#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
case "${1:-}" in
  --execute) args=() ;;
  --dry-run) args=(--dry-run) ;;
  *) echo "用法：$0 --dry-run | --execute"; exit 2 ;;
esac

source scripts/ros_env.sh
b601_source_workspace
plan="$(python3 - <<'PY'
from pathlib import Path
import yaml

cfg = yaml.safe_load(Path("ros2_ws/src/b601_description/config/b601_project.yaml").read_text())
plan = cfg["validation"]["pos_vel_five_poses"]
poses = [",".join(f"{name}={value}" for name, value in zip(cfg["robot"]["arm_joints"], target, strict=True))
         for target in plan["targets"]]
print(";".join(poses))
print(plan["sample_sec"])
PY
)"
mapfile -t trial <<< "$plan"
exec ./scripts/dm_pose_steps.py "${args[@]}" --poses "${trial[0]}" \
  --sample-sec "${trial[1]}"
