#!/usr/bin/env bash
# Source from a B601 script or from a shell at the checkout root.

_b601_checkout_root() {
  (cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
}

_b601_source_setup() {
  local restore_nounset=0 status=0
  if [[ $- == *u* ]]; then
    restore_nounset=1
    set +u
  fi
  source "$1" || status=$?
  if (( restore_nounset )); then
    set -u
  fi
  return "$status"
}

b601_source_ros() {
  local setup="${B601_ROS_SETUP:-}"
  if [[ -z "$setup" ]]; then
    if [[ ${ROS_DISTRO:-} == humble ]] && command -v ros2 >/dev/null 2>&1; then
      return 0
    fi
    if command -v dpkg-query >/dev/null 2>&1; then
      setup="$(dpkg-query -L ros-humble-ros-workspace 2>/dev/null |
        awk '/\/setup\.bash$/ && !found { print; found = 1 }')" || setup=''
    fi
  elif [[ "$setup" != /* ]]; then
    local root
    root="$(_b601_checkout_root)" || return
    setup="$root/$setup"
  fi
  if [[ ! -r "$setup" ]]; then
    printf '%s\n' '请先加载 ROS 2 Humble，或将 B601_ROS_SETUP 设为 ROS 的 setup.bash。' >&2
    return 1
  fi
  _b601_source_setup "$setup" || return
  if [[ ${ROS_DISTRO:-} != humble ]] || ! command -v ros2 >/dev/null 2>&1; then
    printf '%s\n' '加载的环境未提供 ROS 2 Humble。' >&2
    return 1
  fi
}

b601_source_workspace() {
  local root
  root="$(_b601_checkout_root)" || return
  b601_source_ros || return
  local setup="$root/ros2_ws/install/local_setup.bash"
  if [[ ! -r "$setup" ]]; then
    printf '%s\n' '当前工作区尚未构建，请在检出目录运行 ./scripts/build.sh。' >&2
    return 1
  fi
  _b601_source_setup "$setup"
}
