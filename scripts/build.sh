#!/usr/bin/env bash
# build.sh —— 编译工作空间；编完把「默认启动」指向这次构建，并清掉没必要的旧构建
#
#   ./scripts/build.sh              # 编全部包（默认）
#   ./scripts/build.sh b601_safety  # 只编一个包（改 Python 节点时够用）
#   ./scripts/build.sh --clean      # 先删 build/ install/ log/ 再编
#   ./scripts/build.sh --keep-logs  # 保留历史 colcon 日志（默认只留最新一次）
#
# 本脚本负责三条工程约定（见 AGENTS.md §11）：
#   ① 默认启动永远指向最新构建：成功后写 ros2_ws/.last_build 标记并打印启动命令；
#      源码比 install/ 新时 verify.sh 会提示你重编。
#   ② 没有必要就不保留旧构建：清理历史 colcon 日志，删掉 src/ 里已不存在的包的
#      build/ 与 install/ 残留目录（否则 launch 可能抓到已经不存在的包）。
#   ③ 构建失败不更新标记 —— 避免"以为跑的是新版本"。

set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$REPO_ROOT/ros2_ws"
MARKER="$WORKSPACE/.last_build"

CLEAN=0
KEEP_LOGS=0
SELECTED=()

for arg in "$@"; do
  case "$arg" in
    --clean) CLEAN=1 ;;
    --keep-logs) KEEP_LOGS=1 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) SELECTED+=("$arg") ;;
  esac
done

source "$REPO_ROOT/scripts/ros_env.sh"
b601_source_ros

if [[ $CLEAN -eq 1 ]]; then
  echo "==> 清理 build/ install/ log/"
  rm -rf "$WORKSPACE/build" "$WORKSPACE/install" "$WORKSPACE/log"
fi

cd "$WORKSPACE"
ARGS=(--symlink-install)
if [[ ${#SELECTED[@]} -gt 0 ]]; then
  ARGS+=(--packages-select "${SELECTED[@]}")
fi

echo "==> colcon build ${ARGS[*]}"
colcon build "${ARGS[@]}"

# ------------------------------------------------------------------ 清理旧构建
# 只保留最新一次 colcon 日志（每次 build 会生成 log/build_<时间戳>/ 与 log/latest*）
if [[ $KEEP_LOGS -eq 0 && -d "$WORKSPACE/log" ]]; then
  mapfile -t LOG_DIRS < <(find "$WORKSPACE/log" -maxdepth 1 -type d -name 'build_*' | sort)
  if [[ ${#LOG_DIRS[@]} -gt 1 ]]; then
    echo "==> 清理 ${#LOG_DIRS[@]} 份历史构建日志，只留最新一份"
    for dir in "${LOG_DIRS[@]:0:$((${#LOG_DIRS[@]}-1))}"; do
      rm -rf "$dir"
    done
  fi
fi

# 删掉 src/ 里已不存在的包的残留（仅全量构建时做，避免误删只编一个包时的其它包）
#
# ⚠ 血的教训（2026-11-21，见 docs/test-log.md F16）：install/ 根目录下还有 colcon 生成的
#   setup.bash / setup.sh / setup.zsh / setup.ps1 / local_setup.* / _local_setup_util_*.py，
#   只按"名字不在 src/ 里就删"会把它们一起删掉 → 整个工作空间无法 source。
#   所以这里只认"目录"，并且必须是长得像 ROS 包的目录。
#   本工作空间是 colcon isolated 布局（install/<pkg>/…），同时兼容 merged 布局（install/share/<pkg>）。
is_pkg_dir() {
  local dir="$1" name="$2"
  [[ -d "$dir" ]] || return 1
  [[ -d "$dir/share/ament_index/resource_index/packages/$name" ]] && return 0
  [[ -f "$dir/package.xml" ]] && return 0
  [[ -f "$dir/CMakeCache.txt" ]] && return 0
  [[ -f "$dir/colcon_build.rc" ]] && return 0
  return 1
}

if [[ ${#SELECTED[@]} -eq 0 ]]; then
  NON_PACKAGE="ament_index share lib include"
  mapfile -t CANDIDATES < <(
    { ls -1 "$WORKSPACE/build" 2>/dev/null
      ls -1 "$WORKSPACE/install" 2>/dev/null
      ls -1 "$WORKSPACE/install/share" 2>/dev/null; } | grep -v '^\.' | sort -u
  )
  for pkg in "${CANDIDATES[@]}"; do
    [[ -z "$pkg" ]] && continue
    [[ " $NON_PACKAGE " == *" $pkg "* ]] && continue
    [[ -e "$WORKSPACE/src/$pkg" ]] && continue
    # 找准这个候选在哪个位置像个包；三个位置都不像就一律不动
    stale=0
    for base in "$WORKSPACE/build/$pkg" "$WORKSPACE/install/$pkg" "$WORKSPACE/install/share/$pkg"; do
      if is_pkg_dir "$base" "$pkg"; then
        stale=1
        rm -rf "$base"
      fi
    done
    if [[ $stale -eq 1 ]]; then
      echo "==> 删除残留构建：$pkg（src/$pkg 已不存在）"
    fi
  done
fi

# ------------------------------------------------------------------ 更新标记
BUILT_PKGS=$(ls -1 "$WORKSPACE/src" | tr '\n' ' ')
GIT_HEAD=$(cd "$REPO_ROOT" && git rev-parse --short HEAD 2>/dev/null || echo "no-git")
cat > "$MARKER" <<EOF
# 由 scripts/build.sh 自动生成：默认启动以本次构建为准
built_at=$(date -Iseconds)
git_head=$GIT_HEAD
packages=$BUILT_PKGS
install=ros2_ws/install
EOF

cat <<EOF

==> 编译完成。默认启动已指向本次构建（标记文件：ros2_ws/.last_build）

  在检出目录启动网页和模型预览：
    ./scripts/start_target_web.sh

  使用 ROS CLI 前，在新终端的检出目录加载环境：
    source scripts/ros_env.sh && b601_source_workspace
    ros2 launch b601_bringup bringup.launch.py hardware:=mock
    ros2 launch b601_moveit_config demo.launch.py
  改完源码没重编时，./scripts/verify.sh 会提示 install/ 不是最新的。
EOF
