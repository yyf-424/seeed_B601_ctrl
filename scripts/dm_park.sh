#!/usr/bin/env bash
# dm_park.sh —— 收工/停机唯一正确姿势：**先回中、核对到位、再失能**（AGENTS.md §11.4）
#
# 为什么单独写成脚本：§11.4 是硬规则，但它要求"先动电机回中、再看误差、再失能"三步，
# 手工做容易漏（2026-11-21 就因为直接 kill 了 launch，手臂在非竖直位形失能后塌了 ~0.3 rad）。
# 把三步合成一条命令，就没有"忘了"这个失败模式。
#
# 用法（前提：bringup 正在跑，硬件已使能）：
#   ./scripts/dm_park.sh                 # 回中 → 核对 → 失能
#   ./scripts/dm_park.sh --keep-enabled  # 只回中核对，不失能（想接着做实验时用）
#   ./scripts/dm_park.sh --force         # 核对超差也强行失能（会大声警告，慎用）
#   ./scripts/dm_park.sh --duration 25   # 回中用时（默认 20 s）
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# 归中目标来自统一真值的 recenter 段（2026-11-21 实测校准，不要写死）
TRUTH="$REPO_ROOT/ros2_ws/src/b601_description/config/b601_project.yaml"
TARGET="$(python3 - "$TRUTH" <<'PYRT' 2>/dev/null || true
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
r = cfg.get("recenter", {}).get("joints", {})
names = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
if not all(n in r for n in names):
    raise SystemExit(1)
print(",".join("%s=%+.4f" % (n, float(r[n])) for n in names))
PYRT
)"
if [[ -z "$TARGET" ]]; then
  echo "⚠ 真值里没有 recenter.joints，回落到历史常量（0,−0.02,−0.02,0,0,0）"
  TARGET="joint1=0,joint2=-0.02,joint3=-0.02,joint4=0,joint5=0,joint6=0"
fi
DURATION=20
KEEP_ENABLED=0
FORCE=0
ERR_LIMIT=0.02

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2 ;;
    --keep-enabled) KEEP_ENABLED=1; shift ;;
    --force) FORCE=1; shift ;;
    -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
    *) echo "未知参数：$1"; exit 2 ;;
  esac
done

ok()   { printf '\033[1;32m  [OK]\033[0m   %s\n' "$1"; }
bad()  { printf '\033[1;31m  [FAIL]\033[0m %s\n' "$1"; }
info() { printf '\033[1;34m==> %s\033[0m\n' "$1"; }

# --- 0. 环境 ---
if ! command -v ros2 >/dev/null 2>&1; then
  set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; set -u
fi
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "❌ 没有 source ROS 环境（先 source /opt/ros/humble/setup.bash + ros2_ws/install/setup.bash）"
  exit 2
fi
if [[ ! -f "$REPO_ROOT/ros2_ws/install/setup.bash" ]]; then
  echo "❌ 找不到 ros2_ws/install/setup.bash —— 先 ./scripts/build.sh"
  exit 2
fi
set +u; source "$REPO_ROOT/ros2_ws/install/setup.bash" >/dev/null 2>&1; set -u

CM_PID=""
if [[ $KEEP_ENABLED -eq 0 ]]; then
  DEVICE="$(python3 - "$TRUTH" <<'PYDEVICE'
import sys, yaml
with open(sys.argv[1], encoding="utf-8") as config:
    print(yaml.safe_load(config)["hardware"]["dm"]["device"])
PYDEVICE
)"
  read -r -a owners <<< "$(fuser "$DEVICE" 2>/dev/null)"
  if [[ ${#owners[@]} -ne 1 ]] || [[ "$(cat "/proc/${owners[0]}/comm" 2>/dev/null)" != ros2_control_no ]]; then
    bad "无法在 $DEVICE 上确认唯一的控制器进程，现场核对后再回中。"
    exit 2
  fi
  CM_PID="${owners[0]}"
fi
info "1/4 回中"
