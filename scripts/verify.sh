#!/usr/bin/env bash
# verify.sh —— 不需要 sudo 的自检脚本
#
#   ./scripts/verify.sh            # 全套静态检查（默认）
#   ./scripts/verify.sh --build    # 额外跑 colcon build（需要已装 ros2_control / moveit）
#
# 检查内容与顺序：
#   1) 环境：ROS 2、xacro、check_urdf、关键包是否在
#   2) 配置同步：b601_project.yaml（唯一真值）→ 派生文件是否一致
#   3) xacro 展开：mock 与 dm 两个后端都要能展开（这是“改模型没改坏”的最快信号）
#   4) check_urdf：URDF 结构合法性（link 树、关节父子、惯量）
#   5) 跨文件一致性：真值 ↔ URDF ↔ ros2_control ↔ controllers ↔ SRDF ↔ MoveIt ↔ 安全参数
#   6) 文档归档：docs/changes/ 每份架构变化规划文档都要在索引里、且带必需章节
#   7) 安全判定单测：scripts/test_safety.py（不需要 ROS）
#   8) Python 语法：所有节点与 launch 文件 py_compile
#   9) （可选）colcon build + 构建标记
#
# 只读操作：不改环境、不装包、不写仓库（临时文件在 .verify/）。

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$REPO_ROOT/ros2_ws"
DESCRIPTION="$WORKSPACE/src/b601_description"
PROJECT_CONFIG="$DESCRIPTION/config/b601_project.yaml"
TMP="$REPO_ROOT/.verify"
mkdir -p "$TMP"

PASS=0
WARN=0
FAIL=0
ok()   { printf '\033[1;32m  [OK]\033[0m   %s\n' "$*"; PASS=$((PASS+1)); }
warn() { printf '\033[1;33m  [WARN]\033[0m %s\n' "$*"; WARN=$((WARN+1)); }
fail() { printf '\033[1;31m  [FAIL]\033[0m %s\n' "$*"; FAIL=$((FAIL+1)); }
step() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }

# ---------------------------------------------------------------- 1. 环境
step "1/9 环境"

if [[ -f /opt/ros/humble/setup.bash ]] && [[ -z ${ROS_DISTRO:-} ]]; then
  # ROS 的 setup.bash 不是 set -u 安全的，source 期间先关掉 -u
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi
if [[ -n ${ROS_DISTRO:-} ]]; then
  ok "ROS 2 $ROS_DISTRO（$(command -v ros2)）"
else
  fail "没有 ROS 2 环境：先 source /opt/ros/humble/setup.bash"
fi

# xacro 可能来自 ROS 包，也可能是工作区里的 python 环境（本机目前只有后者）
XACRO_CMD=()
if command -v xacro >/dev/null 2>&1; then
  XACRO_CMD=(xacro)
  ok "xacro: $(command -v xacro)"
elif [[ -x "$REPO_ROOT/../.tools/verify-venv/bin/xacro" ]]; then
  export PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH:-}"
  XACRO_CMD=("$REPO_ROOT/../.tools/verify-venv/bin/xacro")
  warn "用工作区 venv 里的 xacro（ROS 包 xacro 尚未安装）：${XACRO_CMD[0]}"
elif python3 -c 'import xacro' 2>/dev/null; then
  XACRO_CMD=(python3 -c 'import xacro; xacro.main()')
  ok "xacro: python 模块"
else
  fail "找不到 xacro：sudo apt install ros-humble-xacro（见 scripts/install_deps.sh）"
fi

if command -v check_urdf >/dev/null 2>&1; then
  ok "check_urdf: $(command -v check_urdf)"
  HAVE_CHECK_URDF=1
else
  warn "没有 check_urdf（ros-humble-urdfdom 通常随 desktop 安装），跳过结构校验"
  HAVE_CHECK_URDF=0
fi

# 一次性取包列表。
# ⚠ 不能写成 `ros2 pkg list | grep -qx "$pkg"`：本脚本开了 pipefail，
#    grep -q 命中后立刻退出会让 ros2 收到 SIGPIPE(141)，整条管道被判失败，
#    于是"装了的包"全被误报成没装。先把列表存进变量即可回避。
PKG_LIST="$(ros2 pkg list 2>/dev/null || true)"
has_pkg() { grep -qxF "$1" <<<"$PKG_LIST"; }

for pkg in ros2_control controller_manager moveit_ros_move_group control_msgs moveit_msgs \
           diagnostic_msgs controller_manager_msgs; do
  if has_pkg "$pkg"; then
    ok "包已安装：$pkg"
  else
    warn "缺包：$pkg（装法见 scripts/install_deps.sh；缺它就无法 build / 运行时验证）"
  fi
done

# -------------------------------------------------------- 2. 配置同步（真值 → 派生）
step "2/9 配置同步（b601_project.yaml → 派生文件）"
if [[ ! -f "$PROJECT_CONFIG" ]]; then
  fail "找不到唯一真值配置：$PROJECT_CONFIG"
elif python3 "$REPO_ROOT/scripts/sync_config.py" --check 2>&1 | sed 's/^/         /'; then
  if [[ ${PIPESTATUS[0]} -eq 0 ]]; then
    ok "派生配置与真值同步（并已通过配置自检）"
  else
    fail "派生配置不同步：跑 ./scripts/sync_config.py 后重试"
  fi
else
  fail "配置自检未通过（见上面的输出）"
fi

# 关键派生文件必须存在（第一次 clone 后忘记 sync 的典型症状）
for f in \
    "$DESCRIPTION/config/initial_positions.yaml" \
    "$WORKSPACE/src/b601_bringup/config/b601_controllers.yaml" \
    "$WORKSPACE/src/b601_moveit_config/config/joint_limits.yaml" \
    "$WORKSPACE/src/b601_safety/config/safety_params.yaml" \
    "$WORKSPACE/src/b601_bringup/config/b601_mit_controllers.yaml"; do
  if [[ -f "$f" ]]; then
    ok "派生文件存在：${f#"$REPO_ROOT"/}"
  else
    fail "缺派生文件：${f#"$REPO_ROOT"/}（跑 ./scripts/sync_config.py）"
  fi
done

# ---------------------------------------------------- 3. xacro 展开（两个后端）
step "3/9 xacro 展开"

EXPANDED=()
if [[ ${#XACRO_CMD[@]} -gt 0 ]]; then
  for hw in mock dm; do
    out="$TMP/b601_${hw}.urdf"
    if "${XACRO_CMD[@]}" \
        "$DESCRIPTION/urdf/b601_dm.urdf.xacro" \
        "hardware:=$hw" \
        "initial_positions_file:=$DESCRIPTION/config/initial_positions.yaml" \
        "project_config_file:=$PROJECT_CONFIG" \
        > "$out" 2> "$TMP/xacro_${hw}.err"; then
      ok "hardware:=$hw 展开成功（$(grep -c '<link ' "$out") link / $(wc -l < "$out") 行）"
      EXPANDED+=("$out")
    else
      fail "hardware:=$hw 展开失败："
      sed 's/^/         /' "$TMP/xacro_${hw}.err" | head -10
    fi
  done
  # MIT 力控模式（effort 命令接口）单独展开一份：检查「每个可控关节都有属主」需要它
  if "${XACRO_CMD[@]}" \
      "$DESCRIPTION/urdf/b601_dm.urdf.xacro" \
      "hardware:=dm" "dm_command_mode:=mit_impedance" \
      "initial_positions_file:=$DESCRIPTION/config/initial_positions.yaml" \
      "project_config_file:=$PROJECT_CONFIG" \
      > "$TMP/b601_mitimpedance.urdf" 2> "$TMP/xacro_mitimpedance.err"; then
    ok "mit_impedance 展开成功（电机侧 MIT 命令接口）"
  else
    fail "mit_impedance 展开失败："
    sed 's/^/         /' "$TMP/xacro_mitimpedance.err" | head -10
  fi
fi

# ------------------------------------------------------------ 4. check_urdf
step "4/9 URDF 结构"
if [[ $HAVE_CHECK_URDF -eq 1 && -f "$TMP/b601_mock.urdf" ]]; then
  if check_urdf "$TMP/b601_mock.urdf" > "$TMP/check_urdf.txt" 2>&1; then
    ok "check_urdf 通过：$(grep -o 'root Link: [a-z_0-9]*' "$TMP/check_urdf.txt")"
  else
    fail "check_urdf 失败："
    sed 's/^/         /' "$TMP/check_urdf.txt" | head -10
  fi
else
  warn "跳过（缺少 check_urdf 或 URDF 未生成）"
fi

# --------------------------------------------------- 5. 跨文件一致性
step "5/9 跨文件一致性（真值 ↔ URDF ↔ 控制器 ↔ SRDF）"
if [[ -f "$TMP/b601_mock.urdf" ]]; then
  if python3 "$REPO_ROOT/scripts/check_consistency.py" "$TMP/b601_mock.urdf" \
      --project-config "$PROJECT_CONFIG" \
      --repo-root "$REPO_ROOT" \
      --controllers "$WORKSPACE/src/b601_bringup/config/b601_controllers.yaml" \
      --moveit-controllers "$WORKSPACE/src/b601_moveit_config/config/moveit_controllers.yaml" \
      --srdf "$WORKSPACE/src/b601_moveit_config/config/b601_dm.srdf" \
      --mit-controllers "$WORKSPACE/src/b601_bringup/config/b601_mit_controllers.yaml" \
      2>&1 | sed 's/^/         /'; then
    ok "一致性检查通过"
  else
    fail "一致性检查未通过（见上面的 [FAIL] 行）"
  fi
else
  warn "跳过（没有展开后的 URDF）"
fi
# MIT 实例覆盖：在 mit_impedance 模型上再查一遍「每关节都有属主」
if [[ -f "$TMP/b601_mitimpedance.urdf" ]]; then
  if python3 "$REPO_ROOT/scripts/check_consistency.py" "$TMP/b601_mitimpedance.urdf" \
      --project-config "$PROJECT_CONFIG" \
      --repo-root "$REPO_ROOT" \
      --mit-controllers "$WORKSPACE/src/b601_bringup/config/b601_mit_controllers.yaml" \
      2>&1 | sed 's/^/         /'; then
    ok "MIT 实例覆盖检查通过（joint1..6 + finger_left 各有属主）"
  else
    fail "MIT 实例覆盖检查未通过（有裸关节 → 力矩恒 0）"
  fi
fi

# ------------------------------------------- 6. 文档归档（docs/changes/）
step "6/9 文档归档（docs/changes/）"
CHANGES_DIR="$REPO_ROOT/docs/changes"
# 开发文档按约定**不纳入 git**（只留本地）；本地没有 docs/ 时跳过本步而不是报错，
# 这样别人 clone 到一份"只有代码"的仓库也能跑通静态自检。
if [[ ! -d "$CHANGES_DIR" ]]; then
  warn "本地没有 docs/changes/（开发文档不纳入 git）—— 跳过归档索引校验"
else
if [[ ! -d "$CHANGES_DIR" ]]; then
  fail "缺 docs/changes/ 目录：架构变化必须有规划文档（见 AGENTS.md §11.2）"
else
  MISSING_INDEX=()
  MISSING_SECTION=()
  for f in "$CHANGES_DIR"/*.md; do
    [[ -e "$f" ]] || continue
    base="$(basename "$f")"
    [[ "$base" == "README.md" || "$base" == _* ]] && continue
    grep -qF "$base" "$CHANGES_DIR/README.md" || MISSING_INDEX+=("$base")
    # 规划文档至少要写清"方案对比"和"验收标准"两节，否则等于没规划
    if ! grep -q "^## 4\. 方案对比" "$f" || ! grep -q "^## 8\. 验收标准" "$f"; then
      MISSING_SECTION+=("$base")
    fi
  done
  if [[ ${#MISSING_INDEX[@]} -gt 0 ]]; then
    fail "这些规划文档没登记进 docs/changes/README.md 索引：${MISSING_INDEX[*]}"
  else
    ok "docs/changes/ 索引完整（$(find "$CHANGES_DIR" -maxdepth 1 -name '*.md' ! -name 'README.md' ! -name '_*' | wc -l) 份规划文档）"
  fi
  if [[ ${#MISSING_SECTION[@]} -gt 0 ]]; then
    fail "这些规划文档缺『方案对比』或『验收标准』章节：${MISSING_SECTION[*]}"
  else
    ok "规划文档章节完整（方案对比 / 验收标准）"
  fi
  # 反向：索引里点名的文件必须存在（防止改名后留下死链）
  DANGLING=()
  # 只校验 markdown 链接目标（"](xxx.md)"），正文里提到的文件名不算链接
  while read -r ref; do
    [[ -z "$ref" ]] && continue
    [[ -f "$CHANGES_DIR/$ref" ]] || DANGLING+=("$ref")
  done < <(grep -oE '\]\([0-9]{4}-[0-9]{2}-[0-9]{2}-[0-9]{2}-[a-z0-9-]+\.md\)' \
             "$CHANGES_DIR/README.md" | sed -E 's/^\]\(//; s/\)$//' | sort -u)
  if [[ ${#DANGLING[@]} -gt 0 ]]; then
    fail "docs/changes/README.md 索引里有失效链接：${DANGLING[*]}"
  else
    ok "索引链接无死链"
  fi
fi

# ------------------------------------------------------- 7. 安全判定单测
fi
step "7/9 安全判定单测（scripts/test_safety.py）"
if python3 "$REPO_ROOT/scripts/test_safety.py" 2>&1 | tail -3 | sed 's/^/         /'; then
  ok "安全判定矩阵全部通过（阈值取自生成的 safety_params.yaml）"
else
  fail "安全判定单测未通过：安全节点可能误停或漏停"
fi

# -------------------------------------------------------------- 7. 语法
step "8/9 Python 语法"
PY_FILES=$(find "$WORKSPACE/src" -name "*.py" -not -path "*/build/*" | sort)
if python3 -m py_compile $PY_FILES 2> "$TMP/py_compile.err"; then
  ok "$(echo "$PY_FILES" | wc -l) 个 Python 文件语法正确"
else
  fail "py_compile 报错："
  sed 's/^/         /' "$TMP/py_compile.err" | head -10
fi
find "$WORKSPACE/src" -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null

# -------------------------------------------------------------- 8. build
step "9/9 colcon build"
if [[ ${1:-} == "--build" ]]; then
  if has_pkg moveit_ros_move_group; then
    # 走 build.sh 而不是裸 colcon：它负责"默认启动指向最新构建"和清理旧构建
    if "$REPO_ROOT/scripts/build.sh" 2>&1 | tail -6 | sed 's/^/         /'; then
      ok "colcon build 成功（记得 source ros2_ws/install/setup.bash）"
    else
      fail "colcon build 失败"
    fi
  else
    fail "缺 moveit / ros2_control，无法 build：先跑 scripts/install_deps.sh"
  fi
else
  # 不编译时至少要回答一个问题：现在 source 的 install/ 是不是最新构建？
  if [[ ! -f "$WORKSPACE/install/setup.bash" ]]; then
    warn "还没构建过：先跑 ./scripts/build.sh（之后 source ros2_ws/install/setup.bash）"
  else
    NEWER=$(find "$WORKSPACE/src" \( -name '*.py' -o -name '*.xacro' -o -name '*.yaml' \
              -o -name '*.xml' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.srdf' \) \
              -newer "$WORKSPACE/install/setup.bash" 2>/dev/null | head -3)
    if [[ -n "$NEWER" ]]; then
      warn "源码比当前 install/ 新 —— 默认启动仍是旧构建，请重跑 ./scripts/build.sh："
      echo "$NEWER" | sed 's/^/         /'
    else
      ok "install/ 已是最新构建（ros2_ws/.last_build 与源码一致）"
    fi
  fi
fi

# --------------------------------------------------------------- 汇总
printf '\n\033[1m汇总\033[0m：%d 通过 / %d 警告 / %d 失败\n' "$PASS" "$WARN" "$FAIL"
if [[ $WARN -gt 0 ]]; then
  printf '  警告通常是“某个包还没装”，按 scripts/install_deps.sh 补上即可。\n'
fi
[[ $FAIL -eq 0 ]]
