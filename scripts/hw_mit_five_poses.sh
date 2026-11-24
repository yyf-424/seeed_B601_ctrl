#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

mode="${1:-}"
case "$mode" in
  --simulate|--probe|--readonly|--execute) ;;
  *) echo "用法：$0 --simulate | --probe | --readonly | --execute"; exit 2 ;;
esac

source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
plan=validation.mit_five_poses
out="$(python3 scripts/project_config.py "$plan.hardware_retest.output_dir")"
mkdir -p "$out"
exec {trial_lock}>"$out/run.lock"
flock --nonblock "$trial_lock" || { echo "已有五姿态测试在运行。" >&2; exit 1; }
export ROS_LOCALHOST_ONLY=1 RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS

if [[ $mode == --simulate ]]; then
  export ROS_DOMAIN_ID="$(python3 scripts/project_config.py "$plan.ros_domain_id")"
  rm -f "$out/sim/results.json"
  controllers="$(ros2 pkg prefix b601_bringup)/share/b601_bringup/config/b601_mit_controllers.yaml"
  exec python3 scripts/mit_five_poses.py --output "$out/sim" --controllers-file "$controllers"
fi

export ROS_DOMAIN_ID="$(python3 scripts/project_config.py "$plan.hardware_retest.ros_domain_id")"
device="$(python3 scripts/project_config.py hardware.dm.device)"
test -c "$device" && command -v fuser >/dev/null || { echo "无法检查串口：$device" >&2; exit 1; }
if fuser "$device"; then
  echo "串口占用：$device" >&2
  exit 1
fi

case "$mode" in
  --probe)
    rm -f "$out/probe.json" "$out/readonly_pose.json"
    python3 scripts/dm_probe.py 2>&1 | tee "$out/pre_probe.log"
    python3 scripts/hw_mit_five_poses_probe.py 2>&1 | tee "$out/probe.log"
    ;;
  --readonly)
    rm -f "$out/readonly_pose.json"
    python3 scripts/hw_mit_five_poses.py --readonly 2>&1 | tee "$out/readonly.log"
    ;;
  --execute)
    test -s "$out/sim/results.json" && test -s "$out/probe.json" && test -s "$out/readonly_pose.json" || {
      echo "请先完成本标签的 --simulate、--probe 和 --readonly。" >&2
      exit 1
    }
    python3 scripts/hw_mit_five_poses.py --execute 2>&1 | tee "$out/run.log"
    if fuser "$device"; then exit 1; fi
    python3 scripts/dm_probe.py 2>&1 | tee "$out/post_probe.log"
    ;;
esac
