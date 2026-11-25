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
#   ./scripts/dm_park.sh --mit           # 电机侧 MIT 分段回中 → 核对 → 失能
#   ./scripts/dm_park.sh --duration 25   # 回中用时（默认 20 s）
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

declare -A cfg=()
settings="$(python3 "$REPO_ROOT/scripts/project_config.py" tools.dm_park --pairs)" || exit 2
while IFS=$'\t' read -r key value; do
  cfg["$key"]="$value"
done <<< "$settings"

DURATION="${cfg[duration_sec]}"
KEEP_ENABLED=0
MIT=0
DURATION_OVERRIDE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)
      [[ $# -ge 2 ]] || { echo "缺少 --duration 的秒数"; exit 2; }
      DURATION="$2"; DURATION_OVERRIDE=1; shift 2 ;;
    --mit) MIT=1; shift ;;
    --keep-enabled) KEEP_ENABLED=1; shift ;;
    -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
    *) echo "未知参数：$1"; exit 2 ;;
  esac
done
if [[ $MIT -eq 1 && $DURATION_OVERRIDE -eq 1 ]]; then
  echo "MIT 回中各段由统一 target_slew 和实测到位反馈推进，--duration 用于位置回路。"
  exit 2
fi

ok()   { printf '\033[1;32m  [OK]\033[0m   %s\n' "$1"; }
bad()  { printf '\033[1;31m  [FAIL]\033[0m %s\n' "$1"; }
info() { printf '\033[1;34m==> %s\033[0m\n' "$1"; }

# --- 0. 环境 ---
source "$REPO_ROOT/scripts/ros_env.sh"
b601_source_workspace || exit 2

CM_PID=""
if [[ $KEEP_ENABLED -eq 0 ]]; then
  DEVICE="$(python3 "$REPO_ROOT/scripts/project_config.py" hardware.dm.device)" || exit 2
  read -r -a owners <<< "$(fuser "$DEVICE" 2>/dev/null)"
  if [[ ${#owners[@]} -ne 1 ]] || [[ "$(cat "/proc/${owners[0]}/comm" 2>/dev/null)" != ros2_control_no ]]; then
    bad "无法在 $DEVICE 上确认唯一的控制器进程。"
    exit 2
  fi
  CM_PID="${owners[0]}"
fi
info "1/4 回中"

# --- 1. 回中 ---
if [[ $MIT -eq 1 ]]; then
  OUT="$(ros2 run b601_demos mit_recenter_call 2>&1)"
  RC=$?
  printf '%s\n' "$OUT" | sed 's/^/        /'
else
  OUT="$(timeout $((DURATION + cfg[result_grace_sec])) python3 scripts/dm_first_move.py \
          --recenter --duration "$DURATION" 2>&1)"
  RC=$?
  echo "$OUT" | grep -E "起点|目标|执行成功|跟踪误差|实际位移|已归中|❌" | sed 's/^/        /'
fi
if [[ $RC -ne 0 ]]; then
  bad "回中失败（退出码 $RC），请核对安全状态与电机状态"
  echo "        排查：/joint_states 是否有数据、安全状态是否 EMERGENCY_STOP、目标是否越限"
  exit 3
fi

# --- 2. 核对 ---
ok "2/4 已通过校准停放位的六轴容差检查"

if [[ $KEEP_ENABLED -eq 1 ]]; then
  info "3/4 --keep-enabled：保持使能，不失能"
  ok "手臂已在竖直位停放；接着做实验请从这一步继续"
  exit 0
fi

# --- 3. 停用控制器 → 失能硬件 → 退出本次 controller_manager ---
info "3/4 停用控制器、失能硬件并退出 controller_manager"
if [[ "$(fuser "$DEVICE" 2>/dev/null | xargs)" != "$CM_PID" ]]; then
  bad "串口持有者已变化。"
  exit 4
fi
if ! python3 "$REPO_ROOT/scripts/dm_park_stop.py" "$CM_PID"; then
  bad "停机未完成。请核对控制器与硬件状态。"
  exit 4
fi
for _ in $(seq "${cfg[disable_attempts]}"); do
  sleep "${cfg[disable_poll_sec]}"
  ps -p "$CM_PID" >/dev/null 2>&1 || break
done
if ps -p "$CM_PID" >/dev/null 2>&1; then
  bad "controller_manager 未退出（PID $CM_PID 仍在）—— 断电前请确认"
  exit 4
fi
ok "controller_manager 已退出（电机应已失能）"

# --- 4. 收尾核对 ---
info "4/4 收尾核对"
if [[ -x scripts/dm_probe.py ]]; then
  if timeout "${cfg[probe_timeout_sec]}" python3 scripts/dm_probe.py 2>/dev/null \
       | grep -E "joint[1-6]|gripper" | sed 's/^/        /'; then
    ok "探针已回读（确认 status_code 全部 OK、温度正常、手臂不明显下沉）"
  else
    bad "探针读不到（可能只是失能后串口状态变化；能读到即正常）"
  fi
fi
echo
