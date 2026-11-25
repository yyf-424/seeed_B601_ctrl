#!/usr/bin/env bash
# install_deps.sh —— 按组打印安装命令（本脚本**不自己调用 sudo**，请复制命令执行）
#
#   ./scripts/install_deps.sh            # 列出全部要装的包与命令
#   ./scripts/install_deps.sh core       # 只列 core 组
#   ./scripts/install_deps.sh control
#   ./scripts/install_deps.sh moveit
#
# 分组原因：一次装全家桶容易把 apt 搞成半成品，也看不出是哪一组的依赖。
# 它只做一件事：把“当前缺哪些包”查清楚并给出可复制的 apt 命令。

set -uo pipefail

DEPENDENCY_GROUPS=("$@")
[[ ${#DEPENDENCY_GROUPS[@]} -eq 0 ]] && DEPENDENCY_GROUPS=(core control moveit observe)

declare -A PKGS=(
  [core]="ros-humble-xacro ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-rviz2"
  [control]="ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-control-msgs ros-humble-ros2controlcli"
  [moveit]="ros-humble-moveit ros-humble-moveit-configs-utils ros-humble-moveit-setup-assistant"
  [observe]="ros-humble-foxglove-bridge ros-humble-rqt-joint-trajectory-controller"
)

declare -A WHY=(
  [core]="URDF/xacro 展开 + RViz 显示（本项目 1、2 步就需要）"
  [control]="ros2_control 框架 + joint_trajectory_controller + control_msgs"
  [moveit]="MoveIt 2：规划、IK、MotionPlanning 面板"
  [observe]="调试用：foxglove 网页看数据、rqt 看控制器"
)

printf '\033[1mB601 示例项目依赖清单\033[0m\n'
printf '（ROS 2 Humble / Ubuntu 22.04。安装需要 sudo，所以脚本不代跑。）\n'

for g in "${DEPENDENCY_GROUPS[@]}"; do
  if [[ -z ${PKGS[$g]:-} ]]; then
    printf '\n\033[1;33m未知分组 %s（可选：%s）\033[0m\n' "$g" "${!PKGS[*]}"
    continue
  fi
  printf '\n\033[1;34m[%s]\033[0m %s\n' "$g" "${WHY[$g]}"
  read -r -a list <<< "${PKGS[$g]}"
  missing=()
  for p in "${list[@]}"; do
    if dpkg -s "$p" >/dev/null 2>&1; then
      printf '   已装  %s\n' "$p"
    else
      printf '   缺失  %s\n' "$p"
      missing+=("$p")
    fi
  done
  if [[ ${#missing[@]} -gt 0 ]]; then
    printf '\n   \033[1m→ 执行：\033[0m\n     sudo apt update && sudo apt install -y %s\n' "${missing[*]}"
  fi
done

cat <<'EOF'

装完之后：
  ./scripts/verify.sh --build
  source scripts/ros_env.sh && b601_source_workspace
  ros2 launch b601_description display.launch.py      # 先看模型
  ros2 launch b601_bringup bringup.launch.py          # 再通控制
  ros2 launch b601_moveit_config demo.launch.py       # 最后上 MoveIt

真机使用 DmSystemHardware。构建前使用当前 Python 解释器安装 motorbridge，仿真另需 mujoco。
EOF
