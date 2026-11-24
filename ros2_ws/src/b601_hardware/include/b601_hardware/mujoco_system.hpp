// mujoco_system.hpp —— ros2_control 的 MuJoCo 硬件接口（仿真后端）
//
// 为什么要有它：
//   本项目规定"上真机前必须先仿真验证"（AGENTS.md §11.3），但在这之前 `hardware:=mujoco`
//   只是 URDF 里的一个占位插件名 —— 跑不起来，所以"仿真验证"一直退化成
//   `hardware:=mock`（无动力学）+ 独立脚本 `sim_mit_check.py`（有动力学但不接 ROS）。
//   这个插件把两者合起来：**ros2_control 的控制器、安全节点、MoveIt 全都在环**，
//   物理由 MuJoCo 算，于是"力控/限幅/越限急停"这些运行期行为可以在仿真里真跑一遍。
//
// 与 DmSystemHardware 的关系：
//   两者导出**同一套接口约定**（joint1..6 + finger_left/right，command=position 或 effort，
//   state=position/velocity/effort），所以 ③④⑤ 层一行都不用改 —— 换后端只改 `hardware:=`。
//
// 控制方式：
//   command_mode = mit_torque  → 命令是力矩，直接写进 MuJoCo 的力矩执行器（纯力矩，最真实）
//   command_mode = pos_vel/mit → 命令是位置，插件内部做 PD（kp/kd 取 URDF 的 <param>），
//                                再写力矩执行器；带每周期限速（velocity_scale × vel_limit）
//
// 关节与执行器：
//   MJCF 里有 8 个关节（6 臂 + 2 指），但只有 6 个力矩执行器。
//   手指没有执行器 → 按"命令即位置"处理（运动学直通），满足夹爪控制器的测试需求。
//
// 时间：
//   不依赖 ROS 的 /clock。每个 write() 按 controller_manager 传来的 period 前进
//   `period / model.opt.timestep` 个子步，因此仿真与真实控制频率一致（默认 200 Hz）。

#pragma once

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

// 直接 include mujoco 头文件：mjModel/mjData 是 C 结构体（typedef struct mjModel_ …），
// 手写前置声明容易写错名字（第一次就是这么踩的）。本头文件只被 mujoco_system.cpp 使用，
// 且 CMake 只给该目标加 mujoco include path，不会污染其它编译单元。
#include <mujoco/mujoco.h>

namespace b601_hardware
{

class MujocoSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MujocoSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct JointMap
  {
    std::string name;
    int qpos_adr{-1};
    int dof_adr{-1};
    int actuator{-1};          // -1 = 无执行器（手指）
    double kp{0.0};            // pos_vel/mit 模式内部 PD
    double kd{0.0};
    double vel_limit{5.0};
    double torque_limit{0.0};
    double min_pos{0.0};
    double max_pos{0.0};
    bool has_position_limit{false};
    bool has_command{false};
    bool is_finger{false};
    // 运行时
    double init_position{0.0};   // URDF state_interface 的 initial_value（仿真初值，避免起点压在限位上）
    double cmd_position{0.0};
    double cmd_effort{0.0};
    double state_position{0.0};
    double state_velocity{0.0};
    double state_effort{0.0};
    double cmd_sent{0.0};      // 限速基准（上一次真正下发的目标位置）
  };

  static double get_double(
    const std::unordered_map<std::string, std::string> & params, const std::string & key,
    double fallback, const std::string & where);

  bool load_model();
  void step_sim(double dt);
  void mirror_finger();

  // 配置
  std::string model_path_;
  std::string command_mode_{"pos_vel"};
  double velocity_scale_{1.0};        // 位置模式下的每周期限速缩放（1.0 = 不额外收紧）
  // 力矩命令符号：与真机插件保持**同一约定**。
  // b601_mit_controller 输出的 τ_cmd 是"−g"约定，dm 插件靠 effort_sign=−1 翻成关节正方向；
  // 仿真后端如果直接用原值，重力前馈就会变成"重力放大"——2026-11-21 实测：
  // 纯 MIT 启动 170 ms 内 joint3 冲到限位（约 27 N·m 满力矩反向）。
  double effort_sign_{-1.0};
  // mit_torque 模式下，"控制器还没接管"之前不要推进物理：
  // 真机在控制器未激活时电机未使能/抱闸，而仿真若此刻就积分，手臂会自由落体扑到限位上，
  // 安全节点第一帧就判急停。收到第一份非零力矩命令（或超过 300 个周期）后才开始积分。
  bool sim_started_{false};
  long cycles_since_activate_{0};
  bool verbose_{false};

  mjModel * model_{nullptr};
  mjData * data_{nullptr};
  std::vector<JointMap> joints_;
  std::size_t finger_left_index_{static_cast<std::size_t>(-1)};
  std::size_t finger_right_index_{static_cast<std::size_t>(-1)};
  rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)};
  long steps_total_{0};
};

}  // namespace b601_hardware
