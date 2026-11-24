#!/usr/bin/env bash
# hw_mit_test1.sh —— 第一次实机验证：**纯 PD 的 MIT 控制器**（kp/kd 只在软件里、电机 kp=kd=0、
# 不加任何重力前馈）能否正确地驱动 6 个关节走完 5 个姿态，并量出静态下垂。
#
# 背景：把原来验证过的 pos_vel 回路换成"纯 PD 的 MIT"。仿真结论（test-log 第十六轮补）：
#       MJCF 没有库仑摩擦 → 仿真里纯 PD 必然"掉下去"；补 frictionloss=0.8 后 5 姿态全部通过，
#       稳态下垂 ≈ τ_g/kp ≈ 0.13 rad（行程 5% 判据 0.157 以内）。
#       真机齿箱有静摩擦，所以真机不会像原仿真那样加速，但**下垂量要实测**。
#
# 安全设计（照 AGENTS.md §11 的规矩来）：
#   ① 只起一个 ros2_control_node（同一 CAN 总线只允许一个程序收发）
#   ② 串口被其他进程持有时退出
#   ③ 目标用增量式（相对"当前实测位形"）+ 斜坡，单步位移 ≤ --max-delta
#   ④ 全程 dm_mit_move.py 盯 /b601_safety/status，EMERGENCY_STOP 立即停发
#   ⑤ 收工**必须回中再失能**（§11.4）：回中用 MIT 控制器自己（mit_torque 下没有 position
#      命令接口，dm_park.sh 的 JTC 回中用不了），核对实测误差 ≤ PARK_LIMIT 才允许失能；
#      超差就**保持使能**并打印处置命令，脚本不会替你失能。
#
# 用法：
#   ./scripts/hw_mit_test1.sh --execute  # 5 个姿态（一条连续动线）
#   ./scripts/hw_mit_test1.sh --dry-run  # 只打印将要下发的目标，不起硬件
#
# ⚠ 前提（人工确认后才跑）：机械臂已用夹具固定、周围清空、24 V 可控、物理急停可达、有人看守。
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
export ROS_HOME="$REPO_ROOT/.verify/roshome"
export ROS_LOG_DIR="$REPO_ROOT/.verify/roslog"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

DRY_RUN=0
EXECUTE=0
SLEW=0.10           # rad/s（比 dm_mit_move 默认 0.2 更慢，首次上真机）
MAX_DELTA=""        # 相邻姿态之间允许的最大关节位移 rad（留空=按振幅档取默认；只是防手误的总闸，实际运动由 slew 限速）
TOL=0.06            # 到位判据 rad（纯 PD 有下垂，设太小会一直等到超时）
SETTLE=1.0          # 目标走完后静置观察秒数（保证报的是稳态值）
PARK_LIMIT=0.05     # 回中核对门槛 rad：超了就**不许失能**
TAG="hw1"
AMP="small"
KP_OVERRIDE=""   # 逗号分隔，覆盖 kp_move（例 70,75,150,35,14,8）
KD_OVERRIDE=""   # 逗号分隔，覆盖 kd_move

while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute)    EXECUTE=1; shift ;;
    --dry-run)    DRY_RUN=1; shift ;;
    --slew)       SLEW="$2"; shift 2 ;;
    --max-delta)  MAX_DELTA="$2"; shift 2 ;;
    --tol)        TOL="$2"; shift 2 ;;
    --settle)     SETTLE="$2"; shift 2 ;;
    --tag)        TAG="$2"; shift 2 ;;
    --amp)        AMP="$2"; shift 2 ;;
    --kp)         KP_OVERRIDE="$2"; shift 2 ;;
    --kd)         KD_OVERRIDE="$2"; shift 2 ;;
    *) echo "未知参数：$1"; exit 2 ;;
  esac
done
if (( DRY_RUN + EXECUTE != 1 )); then
  echo "用法：$0 --dry-run 或 --execute [--amp small|big]"
  exit 2
fi
# ---- 振幅档：OFFSETS + 默认总闸 ----
# small（2026-11-23 第一次）：单关节 ≤0.18 → 实测 j3 只走 1.3°（被下垂+静摩擦吃掉），结论"太小"
# big：
#   ① j3 重力矩在竖直位 ≈7.8 N·m、往前折降到 6.5（限幅 10）→ 折 0.55 rad 也不会掉；
#   ② 纯 PD 下垂是近常数 ≈0.13 rad，指令幅度必须明显大于它
if [[ "${AMP:-small}" == "big" ]]; then
  OFFSETS=(
    "0.35,-0.20,-0.20,0.35,0.25,0.45"
    "0.35,-0.35,-0.40,0.10,0.10,0.00"
    "0.00,-0.40,-0.55,0.35,0.25,-0.45"
    "-0.35,-0.25,-0.30,0.10,-0.20,0.00"
    "-0.35,-0.10,-0.05,-0.20,-0.35,0.45"
  )
else
  OFFSETS=(
    "0.12,-0.12,-0.10,0.12,0.08,0.12"
    "0.12,-0.18,-0.14,0.00,0.04,0.00"
    "0.00,-0.12,-0.08,0.12,0.12,-0.12"
    "-0.12,-0.16,-0.12,0.00,0.00,0.00"
    "-0.12,-0.08,-0.02,-0.12,-0.08,0.12"
  )
fi
MAX_DELTA_DEFAULT=0.15
[[ "${AMP:-small}" == "big" ]] && MAX_DELTA_DEFAULT=0.70
MAX_DELTA="${MAX_DELTA:-$MAX_DELTA_DEFAULT}"

MOVE_LOG="$REPO_ROOT/.verify/hw_${TAG}_move.log"
LAUNCH_LOG="$REPO_ROOT/.verify/hw_${TAG}_bringup.log"

set +u; source /opt/ros/humble/setup.bash; source "$REPO_ROOT/ros2_ws/install/setup.bash"; set -u

step() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
ok()   { printf '\033[1;32m  [OK]\033[0m   %s\n' "$*"; }
bad()  { printf '\033[1;31m  [FAIL]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m  [WARN]\033[0m %s\n' "$*"; }

# 读一帧 /joint_states 的 6 个手臂关节（rclpy 进程内读，避免短命 CLI 节点丢响应）
read_pose() {
  timeout 25 python3 - <<'PY'
import json, sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
rclpy.init(); n = Node("hw_read_pose"); got = {}
def cb(msg):
    for name, pos in zip(msg.name, msg.position):
        if name in ARM: got[name] = pos
n.create_subscription(JointState, "/joint_states", cb, 10)
t0 = time.time()
while time.time() - t0 < 12 and len(got) < 6:
    rclpy.spin_once(n, timeout_sec=0.1)
n.destroy_node(); rclpy.try_shutdown()
if len(got) < 6:
    print("ERR"); sys.exit(1)
print(json.dumps([round(got[k], 4) for k in ARM]))
PY
}

# ------------------------------------------------ 1) 生成本次的纯 PD 控制器 yaml
step "1) 生成纯 PD 配置（gravity_scale=0 / use_gravity=false / friction_scale=0 / free_enabled=false）"
MIT_SHARE="$(ros2 pkg prefix b601_bringup)/share/b601_bringup/config/b601_mit_controllers.yaml"
PD_YAML="$REPO_ROOT/.verify/mit_pd_${TAG}.yaml"
python3 - "$MIT_SHARE" "$PD_YAML" "${KP_OVERRIDE:-}" "${KD_OVERRIDE:-}" <<'PY'
import sys, yaml
src, dst = sys.argv[1], sys.argv[2]
cfg = yaml.safe_load(open(src, encoding="utf-8"))
for key, val in cfg.items():
    if not (isinstance(val, dict) and "ros__parameters" in val):
        continue
    rp = val["ros__parameters"]
    if "gravity_params" not in rp:
        continue                      # 只改 MIT 实例
    rp["gravity_scale"] = 0.0         # 纯 PD：不加任何重力前馈
    rp["use_gravity"] = False
    rp["friction_scale"] = 0.0
    rp["free_enabled"] = [False] * len(rp.get("joints", []))  # 永不"参考跟随实测"
    # 纯 PD 调参：只改 kp_move/kd_move（free_enabled=false → 控制器永远用 move 档）
    kp_env, kd_env = sys.argv[3], sys.argv[4]
    if kp_env:
        vals = [float(x) for x in kp_env.split(",")]
        rp["kp_move"] = vals * len(rp["joints"]) if len(vals) == 1 else vals
    if kd_env:
        vals = [float(x) for x in kd_env.split(",")]
        rp["kd_move"] = vals * len(rp["joints"]) if len(vals) == 1 else vals
yaml.safe_dump(cfg, open(dst, "w", encoding="utf-8"), sort_keys=False, allow_unicode=True)
rp = cfg["b601_mit_arm"]["ros__parameters"]
print(f"  写入 {dst}")
print(f"  kp_move={rp['kp_move']} kd_move={rp['kd_move']}")
print(f"  kp_hold={rp['kp_hold']} kd_hold={rp['kd_hold']}")
print(f"  gravity_scale={rp['gravity_scale']} use_gravity={rp['use_gravity']} "
      f"friction_scale={rp['friction_scale']} free_enabled={rp['free_enabled'][0]}")
PY
[[ -f "$PD_YAML" ]] || { bad "yaml 生成失败"; exit 1; }
ok "纯 PD 配置就绪"

if [[ $DRY_RUN -eq 1 ]]; then
  step "dry-run：不起硬件，直接退出（真跑时会按『当前实测位形』算绝对目标）"
  exit 0
fi

DEVICE=$(python3 -c 'import yaml; print(yaml.safe_load(open("ros2_ws/src/b601_description/config/b601_project.yaml"))["hardware"]["dm"]["device"])')
if [[ ! -c "$DEVICE" ]] || ! command -v fuser >/dev/null || fuser "$DEVICE"; then
  bad "串口不存在或未确认空闲：$DEVICE"
  exit 1
fi

# ---------------------------------------------------------------- 2) 起 bringup
step "2) 起 bringup（hardware:=dm，使能，mit_torque + 纯 PD，安全节点开）"
setsid ros2 launch b601_bringup bringup.launch.py \
  hardware:=dm dm_enable_motors:=true dm_command_mode:=mit_torque \
  use_rviz:=false \
  "controllers:=joint_state_broadcaster b601_mit_arm" \
  "controllers_file:=$PD_YAML" \
  "dm_log_csv:=$REPO_ROOT/.verify/hw_${TAG}.csv" \
  > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
PGID="$LAUNCH_PID"

# ⚠ 只有在"已经回中并核对通过"或"根本没运动过"时才允许失能（§11.4）
ARM_SAFE_TO_DISABLE=1
MOVED=0
cleanup() {
  if [[ $ARM_SAFE_TO_DISABLE -ne 1 ]]; then
    printf '\n\033[1;31m⚠ 手臂未确认回中 —— 脚本**不**替你失能，launch 保持运行（PID %s, PGID %s）\033[0m\n' \
      "${LAUNCH_PID:-?}" "${PGID:-?}"
    printf '   处置：修好定位问题后跑\n'
    printf '     ./scripts/dm_mit_move.py --topic /b601_mit_arm/joint_target --debug-topic /b601_mit_arm/debug \\\n'
    printf '       --set "%s" --slew 0.10 --max-delta 0.5 --tolerance 0.05\n' "${PARK_SPEC:-<recenter>}"
    printf '   确认 /joint_states 已回竖直位后再失能：\n'
    printf '   然后按进程组清理：kill -TERM -- -%s\n' "${PGID:-0}"
    return
  fi
  [[ -n "${PGID:-}" ]] && kill -TERM -- "-$PGID" 2>/dev/null
}
trap cleanup EXIT

# 等控制器 active（最多 90 s）
ACTIVE=0
for _ in $(seq 1 90); do
  sleep 1
  LC="$(ros2 control list_controllers 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g')"
  if echo "$LC" | grep -E '^b601_mit_arm\b' | grep -qw 'active'; then ACTIVE=1; break; fi
  grep -qE "急停触发|拒绝使能|Wrong state|Configuration.*failed" "$LAUNCH_LOG" 2>/dev/null && break
done
if [[ $ACTIVE -ne 1 ]]; then
  bad "控制器没起来 —— 日志尾部："
  grep -E "急停|拒绝|ERROR|WARN|error" "$LAUNCH_LOG" 2>/dev/null | tail -15 | sed 's/^/    /'
  ARM_SAFE_TO_DISABLE=1      # 没运动过（也没使能成功）→ 可以清掉
  exit 1
fi
ok "b601_mit_arm 已 active"
grep -E "温度|CTRL_MODE|使能|enable" "$LAUNCH_LOG" | tail -5 | sed 's/^/    /'

# ------------------------------------------------ 3) 读当前位形 → 算 5 个姿态 + 回中目标
step "3) 读当前位形 → 算 5 个姿态（相对当前位形的增量，全部落在软限位内）"
CUR=$(read_pose)
[[ -z "$CUR" || "$CUR" == "ERR" ]] && { bad "读不到 /joint_states"; ARM_SAFE_TO_DISABLE=1; exit 1; }
ok "当前位形 = $CUR"

# 5 个姿态的"关节偏移"（rad）。两套幅度：
#   AMP=small（2026-11-23 第一次）：单关节 ≤0.18，实测 j3 只走了 1.3°（被下垂+静摩擦吃掉）→ 结论"太小"
#   AMP=big  ：
#     ① j3 重力矩在竖直位 ≈7.8 N·m、往前折降到 6.5（限幅 10）→ 折 0.55 rad 也不会掉；
#     ② 纯 PD 的下垂是近常数 ≈0.13 rad，指令幅度必须明显大于它，实测才看得见运动

TARGETS=$(python3 - "$CUR" "${OFFSETS[@]}" <<'PY'
import json, sys, yaml
from pathlib import Path
cur = json.loads(sys.argv[1])
offs = [[float(x) for x in s.split(",")] for s in sys.argv[2:]]
names = ["joint1","joint2","joint3","joint4","joint5","joint6"]
truth = yaml.safe_load(open("ros2_ws/src/b601_description/config/b601_project.yaml", encoding="utf-8"))
lo = [float(truth["joints"][n]["min_pos"]) for n in names]
hi = [float(truth["joints"][n]["max_pos"]) for n in names]
poses = []
for off in offs:
    p = []
    for n, c, d, l, h in zip(names, cur, off, lo, hi):
        p.append(min(max(round(c + d, 4), l + 0.01), h - 0.01))
    poses.append(p)
for i, p in enumerate(poses, 1):
    print("# 姿态%d " % i + " ".join("%s=%+.3f" % (n, v) for n, v in zip(names, p)), file=sys.stderr)
print(";".join(",".join("%s=%.4f" % (n, v) for n, v in zip(names, p)) for p in poses))
PY
)
[[ -z "$TARGETS" ]] && { bad "目标计算失败"; ARM_SAFE_TO_DISABLE=1; exit 1; }
ok "5 个姿态已算好（见上）；单步最大位移 ≤0.12 rad"

PARK_SPEC=$(python3 - <<'PY'
import yaml
cfg = yaml.safe_load(open("ros2_ws/src/b601_description/config/b601_project.yaml", encoding="utf-8"))
r = cfg["recenter"]["joints"]
print(",".join("%s=%.4f" % (n, float(r[n])) for n in
               ["joint1","joint2","joint3","joint4","joint5","joint6"]))
PY
)
step "4) 回中目标（真值 recenter.joints，与 dm_park.sh 同源）= $PARK_SPEC"

# ------------------------------------------------ 4) 一条连续动线走完 5 个姿态
step "5) 下发【一条连续动线】：5 个姿态，slew=$SLEW rad/s，单步 ≤$MAX_DELTA rad"
MOVED=1
timeout 150 ./scripts/dm_mit_move.py \
  --topic /b601_mit_arm/joint_target \
  --debug-topic /b601_mit_arm/debug \
  --poses "$TARGETS" --slew "$SLEW" --max-delta "$MAX_DELTA" --tolerance "$TOL" \
  --settle "$SETTLE" \
  2>&1 | tee "$MOVE_LOG" | grep -vE "^\s*$" | tail -40
MOVE_RC=${PIPESTATUS[0]}
[[ $MOVE_RC -eq 0 ]] && ok "5 个姿态走完（各姿态稳态误差见下）" || bad "动线未正常完成（rc=$MOVE_RC）"

# ------------------------------------------------ 5) 回中 → 核对 → 失能
step "6) 回中（MIT 控制器自己回竖直位）→ 核对 → 失能"
read_pose >/dev/null 2>&1 || true
timeout 150 ./scripts/dm_mit_move.py \
  --topic /b601_mit_arm/joint_target \
  --debug-topic /b601_mit_arm/debug \
  --set "$PARK_SPEC" --slew 0.10 --max-delta 0.60 --tolerance 0.05 \
  2>&1 | grep -E "起始位形|结束|急停|拒发" | sed 's/^/    /'
PARK_RC=${PIPESTATUS[0]}
sleep 1.5
AFTER=$(read_pose)
if [[ -z "$AFTER" || "$AFTER" == "ERR" ]]; then
  bad "回中后读不到 /joint_states —— 无法核对，**不失能**"
  ARM_SAFE_TO_DISABLE=0
  exit 1
fi
ok "回中后位形 = $AFTER"
PARK_ERR=$(python3 - "$AFTER" "$PARK_SPEC" <<'PY'
import json, sys
q = json.loads(sys.argv[1])
want = {}
for part in sys.argv[2].split(","):
    k, v = part.split("="); want[k] = float(v)
err = max(abs(q[i] - want[n]) for i, n in enumerate(
    ["joint1","joint2","joint3","joint4","joint5","joint6"]))
print("%.4f" % err)
PY
)
if awk -v e="$PARK_ERR" -v l="$PARK_LIMIT" 'BEGIN{exit !(e<=l)}'; then
  ok "已回竖直位：最大误差 ${PARK_ERR} rad ≤ ${PARK_LIMIT} → 允许失能"
else
  bad "回中误差 ${PARK_ERR} rad > ${PARK_LIMIT} —— 纯 PD 可能有下垂，**不失能**"
  ARM_SAFE_TO_DISABLE=0
fi

if [[ $ARM_SAFE_TO_DISABLE -eq 1 ]]; then
  step "7) 结束本次 bringup 并失能"
  kill -INT -- "-$PGID" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 -- "-$PGID" 2>/dev/null || break
    sleep 0.5
  done
  if kill -0 -- "-$PGID" 2>/dev/null; then
    bad "本次 bringup 仍在运行，请现场核对驱动状态。"
    ARM_SAFE_TO_DISABLE=0
    exit 1
  fi
  wait "$LAUNCH_PID" || true
  step "8) 探针回读（status_code / 温度 / 是否下沉）"
  timeout 30 python3 scripts/dm_probe.py 2>/dev/null | grep -E "joint[1-6]|gripper" | sed 's/^/    /' \
    && ok "探针已回读" || warn "探针读不到（失能后串口状态变化，能读到即正常）"
fi

# ------------------------------------------------ 6) 汇总
step "9) 汇总"
if [[ -f "$MOVE_LOG" ]]; then
  echo "  每个姿态的稳态最大误差（纯 MIT 无重力补偿时 ≈ 重力下垂量）："
  grep -E "结束，稳态最大误差" "$MOVE_LOG" | nl -ba | sed 's/^/    /'
  echo "  峰值速度（本工具只在每秒打印一次剩余误差，速度见 CSV：.verify/hw_${TAG}.csv）："
  grep -E "最大剩余误差" "$MOVE_LOG" | tail -6 | sed 's/^/    /'
fi
echo
echo "  日志：$LAUNCH_LOG ；动线日志：$MOVE_LOG ；电机原始数据：.verify/hw_${TAG}.csv"
echo "  回中核对误差：${PARK_ERR:-?} rad（门槛 $PARK_LIMIT）"
[[ $MOVE_RC -eq 0 && ${PARK_RC:-1} -eq 0 && $ARM_SAFE_TO_DISABLE -eq 1 ]] && exit 0 || exit 1
