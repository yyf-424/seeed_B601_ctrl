// dm_system.hpp —— ros2_control 的达妙电机 SystemInterface 实现
//
// 这个类是"第 ② 层硬件层"的唯一入口：controller_manager 按 URDF 里的
//   <plugin>b601_hardware/DmSystemHardware</plugin>
// 把它 dlopen 起来，然后用六个生命周期回调驱动它：
//
//   on_init      → 读 URDF 里的硬件参数与关节参数（配置阶段就固定下来）
//   on_configure → 打开串口、注册电机、收一帧真实反馈（此时仍不使能）
//   on_activate  → 把命令初值设成实测位置，然后（可选）使能电机
//   read         → 每周期：收反馈 → 填 state interface
//   write        → 每周期：把 command interface 写进电机
//   on_deactivate→ 失能电机
//
// 三层安全措施（真机上缺一不可）：
//   1. 激活时命令初值 = 实测位置 → 消除"上电瞬间跳到 0 位"
//   2. 每周期限速（vel_limit × dt）→ 命令突变时不会一步跳到位
//   3. 看门狗（watchdog_timeout 内没有新命令 → 原地保持）→ 上层崩了也不会失控
//
// 参数一览（写在 URDF 的 <hardware><param> 与 <joint><param> 里，见 ros2_control.xacro）
//   硬件级：device / baudrate / command_mode / enable_motors / watchdog_timeout /
//           can_timeout_ms / max_write_failures / feedback_wait_ms
//   关节级：motor_id / feedback_id / model / direction / zero_offset / kp / kd /
//           vel_limit / min_pos / max_pos

#pragma once

#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "b601_hardware/temperature_guard.hpp"

namespace b601_hardware
{

class DmSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DmSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 单个关节 = 一台电机 + 一堆标定/限幅参数
  struct JointConfig
  {
    std::string name;
    int motor_id{0};
    int feedback_id{0};
    std::string model;
    double direction{1.0};      // +1 / -1：关节正方向与电机正方向的关系
    double scale{1.0};          // 电机 rad → 关节单位 的比例（臂关节=1；夹爪 m/rad）
    // 关节 effort ↔ 电机力矩的换算（决策⑫）：
    //   臂（revolute）：关节单位 N·m ↔ 电机力矩 1:1   → effort_scale = 1.0
    //   夹爪（prismatic）：关节单位是 N，电机是 N·m，靠齿条-齿轮 r 换算 τ = F·r
    //   r 由位置换算比例直接给出：scale = 0.01 m/rad ⇒ 运动 1 rad 走 0.01 m ⇒ r = 0.01 m
    //   ⚠ 数值上现在等于 scale，但含义不同（位置换算 vs 力矩换算），必须分开给。
    double effort_scale{1.0};
    double zero_offset{0.0};    // 关节 0 位相对电机编码器 0 位的偏置
    double t_mos{0.0};          // 反馈帧自带的 MOS 温度（℃，未换算）
    double t_rotor{0.0};        // 反馈帧自带的转子温度（℃）
    double kp{0.0};             // MIT 模式位置刚度
    double kd{0.0};             // MIT 模式阻尼
    double vel_limit{5.0};      // rad/s，用于限速与 POS_VEL 模式的 vlim
    double torque_limit{0.0};   // N·m，>0 时对力矩命令做限幅（mit_torque 模式用）
    double min_pos{0.0};        // 软限位下限（rad），超出即夹紧
    double max_pos{0.0};        // 软限位上限
    // 静止零位漂移许用值（rad/m，逐关节）：手臂静止顶在机械停止位上时实测值可能略微越界
    // （2026-11-21 实测 joint2 读到 +0.026）。只用于 on_activate 的超限检查，
    // 不参与 write() 的夹紧 —— 命令永远夹在真实限位内。
    double limit_tolerance{0.05};
    bool has_position_limit{false};
    bool has_command{false};    // 该关节是否有命令接口（finger_right 没有）
    int reject_count{0};        // 被合理性检查拦掉的次数
    // 反馈跳变门限（rad/周期）。>0 时用它，0 表示按 vel_limit 自动推算。
    // 为什么要有显式开关：自动公式 = vel_limit·0.08 + 0.05，一旦为了跑快去调大
    // vel_limit，这道"坏帧守卫"会跟着变松。想放宽速度又不想放松守卫，就在这里钉死。
    double feedback_jump_limit{0.0};

    // 运行时
    void * handle{nullptr};
    double cmd{0.0};            // 控制器写进来的期望位置
    double cmd_effort{0.0};
    // mit_impedance（Phase 3）：位置环交给驱动器，控制器下发 (pos, kp, kd, τ_ff)
    double cmd_kp{0.0};
    double cmd_kd{0.0};     // 控制器写进来的期望力矩（mit_torque 模式）
    double command_sent{0.0};   // 上一周期真正下发到电机的值（限速基准）
    double state_pos{0.0};
    double state_vel{0.0};
    double state_effort{0.0};
    bool ever_received{false};
    // 电机侧原始读数（未经方向/比例换算、也未经合理性过滤）。
    // 只用于 CSV 记录：真出"抽风"时，state_vel 可能被守卫丢掉，而这里留着真值。
    double raw_pos_motor{0.0};
    double raw_vel_motor{0.0};
    double raw_torq_motor{0.0};
    bool raw_valid{false};
  };

  // ---- 参数读取 ----
  bool parse_joint_config(std::size_t index, const hardware_interface::ComponentInfo & joint);
  void parse_hardware_parameters();
  static double get_double(
    const std::unordered_map<std::string, std::string> & params, const std::string & key,
    double fallback, const std::string & where);
  static bool get_bool(
    const std::unordered_map<std::string, std::string> & params, const std::string & key,
    bool fallback, const std::string & where);
  static int get_int(
    const std::unordered_map<std::string, std::string> & params, const std::string & key,
    int fallback, const std::string & where);

  // ---- 与 abi 打交道的薄封装 ----
  bool open_bus();
  void close_bus();
  bool wait_for_first_feedback();
  /// 读回电机的 CTRL_MODE 寄存器，确认模式切换真的生效（不生效就拒绝使能）
  bool verify_control_mode(std::uint32_t expected_mode);
  void poll_state();
  void command_motor(JointConfig & joint, double target, double period);

  // ---- CSV 记录（速度/MIT 数据留痕，排查"抽风"用）----
  bool open_log_file();
  void close_log_file();
  void append_log_row(double t_sec);
  void flush_log_buffer();

  // ---- 配置（on_init 填好，之后只读）----
  std::string device_{"/dev/ttyACM0"};
  int baudrate_{921600};
  std::string command_mode_{"pos_vel"};   // pos_vel | mit
  bool enable_motors_{true};              // false = 只读模式（dry run，不使能电机）
  double watchdog_timeout_{0.5};
  // 全局速度缩放（0~1）：同时作用于「每周期限速」与「POS_VEL 的 vlim」。
  // 首次上真机时调到 0.1，确认方向/幅值都对之后再放开。默认就取保守值。
  double velocity_scale_{0.2};

  // 电机力矩符号约定：达妙电机上报/接收的 torq 与「关节正方向力矩」反号
  // （2026-11-20 实测：用 +1 拟合重力会得到负质量）。统一在插件里做一次转换，
  // 这样 /joint_states.effort 与 effort 命令都是"关节约定"，上层不用关心电机细节。
  double effort_sign_{-1.0};
  // 力矩**命令**的符号：与 effort_sign_ **不是同一个**！
  //   · 反馈：电机上报的 torq 与关节正方向力矩反号 → effort_sign_ = −1（2026-11-20 实测：
  //     用 +τ 拟合重力会得到 7 个负质量，取反后质量全正且接近 CAD）
  //   · 命令：电机的 tau 命令与关节正方向力矩**同号**（2026-11-20 实测：给 joint5 发
  //     +1.0 N·m → joint5 朝正方向走 0.113 rad）
  // 2026-11-21 真机实测：把反馈用的 −1 也用到命令上 → PD 变成正反馈，
  // 首个 0.05 rad 动作中 joint5 冲到 10.26 rad/s，靠安全节点急停才没出事。
  double command_sign_{1.0};
  int can_timeout_ms_{0};
  int max_write_failures_{20};
  int feedback_wait_ms_{2000};
  // 达妙电机是"应答式"的：不主动上报，必须每个周期显式发请求帧（实测：裸读串口完全静默）。
  // feedback_divider=1 表示每个周期都请求；想降总线负载就调大（2 → 每两周期请求一次）。
  bool request_feedback_{true};
  int feedback_divider_{1};
  int mode_switch_timeout_ms_{1000};
  long feedback_tick_{0};

  // CSV 记录：log_csv 为空 = 关闭（默认）。开启后每个采样周期写一行，
  // 列含义见文件头注释；含「电机原始速度」与「MIT 力矩命令」，
  // 正好补上 2026-11-20 抽风事故没留下速度数据的缺口。
  std::string log_csv_{""};
  double log_rate_hz_{200.0};
  double log_period_s_{0.005};
  double log_accum_s_{0.0};
  std::uint64_t log_rows_{0};
  double log_t0_{-1.0};
  std::ofstream log_file_{};

  // 温度守护（决策③）：不做力矩位置保护，堵转就靠温度兜底。
  // 温度来自反馈帧自带的 t_mos / t_rotor —— 插件以前完全没读它们。
  TemperatureGuard temp_guard_;
  bool over_temp_{false};          // 已触发过温失能（锁存，需人工重启硬件组件）
  bool temp_warned_{false};
  std::string log_buffer_{};

  // ---- 运行时 ----
  static constexpr std::size_t kInvalidIndex = static_cast<std::size_t>(-1);

  std::vector<JointConfig> joints_;
  std::size_t finger_left_index_{kInvalidIndex};
  std::size_t finger_right_index_{kInvalidIndex};
  void * controller_{nullptr};
  bool enabled_{false};
  bool bus_open_{false};

  // 节流日志与看门狗用 steady 时钟：不随 use_sim_time 跳变，也不受系统校时影响
  rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)};
  std::chrono::steady_clock::time_point last_command_time_{};
  bool has_last_command_{false};
  rclcpp::Time last_feedback_time_{0, 0, RCL_STEADY_TIME};
  int consecutive_write_failures_{0};
  int consecutive_empty_reads_{0};
  bool watchdog_warned_{false};
};

}  // namespace b601_hardware
