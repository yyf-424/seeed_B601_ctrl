#!/usr/bin/env bash
# preflight.sh —— 上真机前的门禁。**每次接真机、每次上电前先跑这个。**
#
#   ./scripts/preflight.sh                 # 默认：mock 后端跑完整安全链路（无实物、无风险）
#   ./scripts/preflight.sh --keep-running  # 跑完不关 bringup（方便手动接着看）
#
# 它回答一个问题：**现在能不能上真机？** 只有全部 PASS 才允许。
#
# 检查项：
#   1) 配置同步      b601_project.yaml → 派生文件一致
#   2) 安全判定单测  scripts/test_safety.py（阈值来自统一配置）
#   3) 静态自检      ./scripts/verify.sh（xacro / check_urdf / 跨文件一致性）
#   4) 仿真闭环      mock 后端起 bringup + 安全节点，实测：
#        · 启动姿态 → 正常（NORMAL）
#        · 安全范围内离开启动位形 → 警告（WARNING），且控制器**仍在** active
#        · 注入角度越限帧 → 急停（EMERGENCY_STOP），控制器全部 inactive、硬件 INACTIVE
#        · 越限状态下解除急停 → 必须被拒绝
#        · 回到安全范围后解除 → 成功
#   5) MuJoCo 自检    若装了 mujoco：跑 scripts/sim_mit_check.py（动力学级验证）
#
# 为什么用 mock 而不是直接上真机：
#   mock 后端能完整跑控制层与安全链路（控制器、action、/joint_states、急停动作），
#   但没有真实电机，所以可以放心注入越限数据。真机上永远不做这种注入。
#
# 退出码：0 = 可以上真机；非 0 = 禁止上真机。

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$REPO_ROOT/ros2_ws"
TMP="$REPO_ROOT/.verify"
LOG="$TMP/preflight_bringup.log"
mkdir -p "$TMP"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-171}"
export ROS_LOCALHOST_ONLY=1
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS

KEEP_RUNNING=0
[[ ${1:-} == "--keep-running" ]] && KEEP_RUNNING=1

PASS=0
FAIL=0
ok()   { printf '\033[1;32m  [PASS]\033[0m %s\n' "$*"; PASS=$((PASS+1)); }
bad()  { printf '\033[1;31m  [FAIL]\033[0m %s\n' "$*"; FAIL=$((FAIL+1)); }
info() { printf '\033[1;34m  [INFO]\033[0m %s\n' "$*"; }
step() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WORKSPACE/install/setup.bash"
set -u

export ROS_HOME="$TMP/roshome"
export ROS_LOG_DIR="$TMP/roslog"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

LAUNCH_PID=""
cleanup() {
  # ⚠ 只杀"自己这一组"进程（docs/test-log.md F17）：
  #   本机不同 bash 会话的进程彼此可见，用 `pkill -f 节点名` 会误杀别的会话里的 ROS 节点。
  #   launch 用 setsid 起，因此组 ID = LAUNCH_PID，`kill -- -PID` 精确命中它和它的子进程。
  if [[ -n "$LAUNCH_PID" ]]; then
    if [[ $KEEP_RUNNING -eq 1 ]]; then
      info "保留 bringup（进程组 $LAUNCH_PID），用完请 kill -- -$LAUNCH_PID"
    else
      kill -INT -- "-$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null || true
      for _ in $(seq 1 20); do
        kill -0 -- "-$LAUNCH_PID" 2>/dev/null || break
        sleep 0.25
      done
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
      wait "$LAUNCH_PID" 2>/dev/null || true
    fi
  fi
  rec_stop 2>/dev/null || true
  if kill -0 -- "-$LAUNCH_PID" 2>/dev/null; then
    bad "进程组 $LAUNCH_PID 仍在运行，请手动 kill -- -$LAUNCH_PID"
  fi
}
trap cleanup EXIT

# ---------------------------------------------------------------- 1. 配置同步
step "1/5 配置同步"
if python3 "$REPO_ROOT/scripts/sync_config.py" --check >/dev/null 2>&1; then
  ok "派生配置与 b601_project.yaml 同步"
else
  bad "派生配置未同步 → 跑 ./scripts/sync_config.py"
fi

# ------------------------------------------------------------ 2. 安全判定单测
step "2/5 安全判定单测"
if python3 "$REPO_ROOT/scripts/test_safety.py" >"$TMP/preflight_safety_units.log" 2>&1; then
  ok "$(tail -1 "$TMP/preflight_safety_units.log")"
else
  bad "安全判定单测失败 → 见 $TMP/preflight_safety_units.log"
fi

# ---------------------------------------------------------------- 3. 静态自检
step "3/5 静态自检"
if "$REPO_ROOT/scripts/verify.sh" >"$TMP/preflight_verify.log" 2>&1; then
  ok "$(grep -a '汇总' "$TMP/preflight_verify.log" | tail -1 | sed 's/\x1b\[[0-9;]*m//g')"
else
  bad "静态自检未通过 → 见 $TMP/preflight_verify.log"
fi

# ------------------------------------------------------------ 4. 仿真闭环
step "4/5 仿真闭环（mock 后端 + 安全节点）"

wait_topic() {
  local topic="$1" timeout="${2:-30}" i=0
  while (( i < timeout * 2 )); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      return 0
    fi
    sleep 0.5
    i=$((i+1))
  done
  return 1
}

STATUS_LOG="$TMP/preflight_status_stream.log"
STATUS_PID=""
rec_start() {  # 持续录制状态话题，任何一次状态跳变都不会漏
  rec_stop
  : > "$STATUS_LOG"
  # stdbuf -oL：ros2 topic echo 写文件时是块缓冲，不按行刷就会"看起来没收到"
  PYTHONUNBUFFERED=1 ros2 topic echo /b601_safety/status >>"$STATUS_LOG" 2>/dev/null &
  STATUS_PID=$!
  sleep 0.6
}
rec_stop() {
  if [[ -n "$STATUS_PID" ]] && kill -0 "$STATUS_PID" 2>/dev/null; then
    kill "$STATUS_PID" 2>/dev/null || true
    wait "$STATUS_PID" 2>/dev/null || true
  fi
  STATUS_PID=""
}
rec_clear() { : > "$STATUS_LOG"; }
rec_has() { grep -q "value: $1" "$STATUS_LOG"; }
rec_count() { grep -c "value: $1" "$STATUS_LOG" || true; }
status_dump() { timeout 8 ros2 topic echo /b601_safety/status --once 2>/dev/null; }
field_of() {  # field_of "<dump>" state|level
  local dump="$1" key="$2"
  case "$key" in
    level) echo "$dump" | awk '/^level:/{print $2; exit}' ;;
    state) echo "$dump" | awk '/key: state/{getline; print $2; exit}' ;;
  esac
}

controllers_active() {
  # 必须整词匹配：inactive 里也含 "active"；顺便去掉 ANSI 颜色码
  ros2 control list_controllers 2>/dev/null \
    | sed 's/\x1b\[[0-9;]*m//g' | grep -cE '(^| )active( |$)' || true
}

hardware_state() {
  # 输出形如：  state: id=3 label=active
  ros2 control list_hardware_components 2>/dev/null \
    | awk -F'label=' '/state:/{print $2; exit}' || true
}

inject() {  # inject <positions_list> <velocities_list> [秒数] [频率]
  # 注意：ros2 topic pub 光启动就要 1s+，用 timeout 按"持续时长"杀会在发出第一帧前就死。
  # 所以用 --times 控制发多少帧，外层 timeout 只作为兜底。
  local positions="$1" velocities="$2" secs="${3:-1.5}" rate="${4:-200}"
  local times
  times="$(awk "BEGIN{printf \"%d\", $secs*$rate}")"
  timeout 40 ros2 topic pub -r "$rate" -t "$times" /joint_states sensor_msgs/msg/JointState \
    "{name: ['joint1','joint2','joint3','joint4','joint5','joint6','finger_left','finger_right'],
      position: $positions, velocity: $velocities}" >/dev/null 2>&1 || true
  sleep 0.4
}

: > "$LOG"
# setsid：让 launch 及其子节点成为独立进程组，收尾时按组精确清理，不波及其它会话
setsid ros2 launch b601_bringup bringup.launch.py hardware:=mock use_rviz:=false \
  >>"$LOG" 2>&1 &
LAUNCH_PID=$!

if wait_topic /b601_safety/status 40 && wait_topic /joint_states 40; then
  ok "bringup 起来了（/joint_states 与 /b601_safety/status 都在）"
else
  bad "40s 内没看到必要话题 → 见 $LOG"
  tail -20 "$LOG" | sed 's/^/         /'
fi

sleep 2  # 让 spawner 把三个控制器激活

# 预热服务发现：第一次 ros2 service call 要等 DDS 发现（本机实测可达 20s+），
# 先空跑一次把它从关键路径上挪走，后面的调用就快了。
timeout 60 ros2 service call /b601_safety/clear_estop std_srvs/srv/Trigger >/dev/null 2>&1 \
  || info "预热 clear_estop 超时（不影响判定，后面会再调一次）"

# --- 4.1 启动姿态 → NORMAL ---
rec_start
rec_clear
inject "[0.0, -0.30, -0.60, 0.0, 0.0, 0.0, 0.0, 0.0]" "[0,0,0,0,0,0,0,0]" 1.5 200
if rec_has NORMAL; then
  ok "启动姿态 → NORMAL（正常）"
else
  bad "启动姿态应判 NORMAL → 见 $STATUS_LOG"
fi
ACTIVE_BEFORE="$(controllers_active)"
if [[ "$ACTIVE_BEFORE" -ge 3 ]]; then
  ok "控制器 active 数量 = $ACTIVE_BEFORE（joint_state_broadcaster + 手臂 + 夹爪）"
else
  bad "active 控制器只有 $ACTIVE_BEFORE 个（应为 3）"
fi

# --- 4.2 安全范围内离开启动位形 → WARNING，且不许停机 ---
rec_clear
inject "[0.0, -0.30, -1.20, 0.0, 0.0, 0.0, 0.0, 0.0]" "[0,0,0,0,0,0,0,0]" 1.5 200
if ! rec_has WARNING; then
  # DDS/注入偶发丢帧时重试一次，避免"门禁抖动"被误当成功能缺陷
  inject "[0.0, -0.30, -1.20, 0.0, 0.0, 0.0, 0.0, 0.0]" "[0,0,0,0,0,0,0,0]" 1.5 200
fi
if rec_has WARNING; then
  ok "离开启动位形（joint3=-1.20）→ WARNING（警告）"
else
  bad "应判 WARNING → 见 $STATUS_LOG"
fi
ACTIVE_WARN="$(controllers_active)"
if [[ "$ACTIVE_WARN" -eq "$ACTIVE_BEFORE" ]]; then
  ok "警告态控制器仍在 active（$ACTIVE_WARN 个）—— 警告不阻止继续驱动"
else
  bad "警告态把控制器停了（$ACTIVE_BEFORE → $ACTIVE_WARN），与设计不符"
fi

# --- 4.3 角度越限 → EMERGENCY_STOP + 全臂失能 ---
rec_clear
inject "[3.00, -0.30, -0.60, 0.0, 0.0, 0.0, 0.0, 0.0]" "[0,0,0,0,0,0,0,0]" 1.5 200
if ! rec_has EMERGENCY_STOP; then
  # 同一个 DDS 偶发丢帧问题（F15/F22）：注入没送达时重试一次，避免"门禁抖动"掩盖真实缺陷
  info "越限注入没被采到，重试一次"
  inject "[3.00, -0.30, -0.60, 0.0, 0.0, 0.0, 0.0, 0.0]" "[0,0,0,0,0,0,0,0]" 1.5 200
fi
sleep 2.5   # 等异步急停动作链跑完（停控制器 → 硬件 INACTIVE）
if rec_has EMERGENCY_STOP; then
  ok "joint1=3.00（> 2.60 上限）→ EMERGENCY_STOP（急停）"
else
  bad "应判 EMERGENCY_STOP → 见 $STATUS_LOG"
fi
ACTIVE_STOP="$(controllers_active)"
if [[ "$ACTIVE_STOP" -eq 0 ]]; then
  ok "急停后控制器全部 inactive（0 个 active）"
else
  bad "急停后仍有 $ACTIVE_STOP 个控制器 active"
fi
HW="$(hardware_state)"
if [[ "$HW" == *"inactive"* ]]; then
  ok "硬件已置为 inactive（on_deactivate → disable_all，即全臂失能）"
else
  bad "硬件状态未变为 inactive：'$HW'"
fi

# --- 4.4 速度越限同样触发急停；解除急停的拒绝/放行逻辑 ---
CLEAR="ros2 service call /b601_safety/clear_estop std_srvs/srv/Trigger"
out="$(timeout 25 $CLEAR 2>&1)"
if [[ "$out" != *"response:"* ]]; then
  info "第一次 clear_estop 没拿到响应（DDS 发现慢），重试一次"
  out="$(timeout 40 $CLEAR 2>&1)"
fi
if [[ "$out" == *"success=False"* ]]; then
  ok "越限状态下解除急停被拒绝（符合预期）"
else
  bad "越限状态下解除急停应被拒绝，实际：$out"
  info "现场：$(ros2 service list 2>/dev/null | grep -c clear_estop) 个 clear_estop 服务"
fi

# 回到安全范围（也在启动位形附近）后再解除
inject "[-2.40, -0.30, -0.60, 0.0, 0.0, 0.0, 0.0, 0.0]" "[0,0,0,0,0,0,0,0]" 1.5 200
out="$(timeout 25 $CLEAR 2>&1)"
if [[ "$out" != *"response:"* ]]; then
  info "解除急停没拿到响应，重试一次"
  out="$(timeout 40 $CLEAR 2>&1)"
fi
if [[ "$out" == *"success=True"* ]]; then
  ok "回到安全范围后解除急停成功（且不自动恢复控制器）"
else
  bad "回到安全范围后应能解除急停，实际：$out"
fi

rec_clear
inject "[2.40, -0.30, -0.60, 0.0, 0.0, 0.0, 0.0, 0.0]" "[0, 999.0, 0, 0, 0, 0, 0, 0]" 1.5 200
if ! rec_has EMERGENCY_STOP; then
  inject "[2.40, -0.30, -0.60, 0.0, 0.0, 0.0, 0.0, 0.0]" "[0, 999.0, 0, 0, 0, 0, 0, 0]" 1.5 200
fi
if rec_has EMERGENCY_STOP; then
  ok "joint2 角速度 999 rad/s → EMERGENCY_STOP（速度越限同样触发）"
else
  bad "速度越限应判 EMERGENCY_STOP → 见 $STATUS_LOG"
fi
rec_stop
inject "[-2.40, -0.30, -0.60, 0.0, 0.0, 0.0, 0.0, 0.0]" "[0,0,0,0,0,0,0,0]" 0.6 200
timeout 25 $CLEAR >/dev/null 2>&1

# ---------------------------------------------------------------- 5. MuJoCo
step "5/6 MuJoCo 动力学自检（可选）"
if python3 -c "import mujoco" >/dev/null 2>&1; then
  if python3 "$REPO_ROOT/scripts/sim_mit_check.py" >"$TMP/preflight_mujoco.log" 2>&1; then
    ok "MuJoCo 自检通过 → $(tail -1 "$TMP/preflight_mujoco.log" | cut -c1-60)"
  else
    bad "MuJoCo 自检失败 → 见 $TMP/preflight_mujoco.log"
  fi
else
  info "本机没装 mujoco，跳过（pip install --user mujoco）"
  info "注意：hardware:=mujoco 后端仍未实现，仿真验证目前依赖 mock + sim_mit_check.py"
fi

# ------------------------------------------- 6. 变刚度（MIT）全链路回归（仿真）
# 这一节覆盖 Phase 0/1 新增的能力：指令层状态机、整组 done、拆分实例、夹爪力控、过温守护。
# 全部走真正的 ROS 链路（controller_manager → b601_mit_controller → MuJoCo 硬件插件），
# **不碰真机**。任一项失败就禁止上真机 —— 变刚度这条线动的是力矩输出，比位置模式更危险。
step "6/6 变刚度（MIT）全链路回归（MuJoCo，不上真机）"
export ROS_DOMAIN_ID=173
if python3 -c "import mujoco" >/dev/null 2>&1 && [[ -x "$REPO_ROOT/scripts/sim_mit_ros.py" ]]; then
  MIT_LOG="$TMP/preflight_mit.log"
  : >"$MIT_LOG"
  mit_run() {   # $1=说明 $2=tag 其余=参数
    local desc="$1"; local tag="$2"; shift 2
    if timeout 300 python3 "$REPO_ROOT/scripts/sim_mit_ros.py" --tag "$tag" "$@" \
        >"$MIT_LOG.$tag" 2>&1; then
      ok "$desc"
    else
      bad "$desc → 见 $MIT_LOG.$tag"
      grep -E "RESULT|❌" "$MIT_LOG.$tag" | tail -3 | sed 's/^/         /'
    fi
  }
  mit_run "零重力跟踪回归（3 姿态，稳态误差判据 0.02）" pf_track \
      --zero-gravity --no-safety --tol 0.02 --max-vel 1.5
  mit_run "变刚度状态机（slew/死区/整组 done/刚度斜坡）" pf_state \
      --zero-gravity --no-safety --state-machine-test
  mit_run "夹爪 MIT 力控（0/20/40 mm）" pf_grip \
      --controllers "b601_mit_arm b601_mit_gripper" --zero-gravity --no-safety \
      --joints finger_left --poses "0.0;0.02;0.04" --tol 0.004 --max-vel 0.3
  mit_run "过温守护（热模型升温 → 失能）" pf_temp --temp-trip-test --no-safety --tol 99 --max-vel 99
else
  info "跳过：缺 mujoco 或 scripts/sim_mit_ros.py"
fi

# ---------------------------------------------------------------- 汇总
printf '\n\033[1m上电门禁\033[0m：%d 通过 / %d 失败\n' "$PASS" "$FAIL"
if [[ $FAIL -eq 0 ]]; then
  printf '\033[1;32m✅ 允许上真机。\033[0m 流程：先 enable_motors:=false 只读 → 手推核对方向 → 再小幅度使能。\n'
  printf '   本版本复测入口见 README.md。先运行：./scripts/hw_mit_test1.sh --dry-run --amp big --slew 0.12\n'
else
  printf '\033[1;31m⛔ 禁止上真机。\033[0m 先修到全绿，再回到本脚本。\n'
fi
[[ $FAIL -eq 0 ]]
