// b601_mit_controller.hpp —— B601 的 MIT 力控控制器（重力/摩擦前馈 + 增益调度）
//
// 它解决的问题（用户需求原文）：
//   "用一个 mit 控制器，tff 包括重力补偿、摩擦补偿，然后当 pos 和 vel 静止的时候 kp 可以
//    保持很小，维持一个小刚度的机械臂；当前端发送 target 时可以切换为大 kp，快速移动过去"
//
// 控制律（每周期、每关节）：
//   τ_i = Kp_i·(q_des,i − q_i) + Kd_i·(0 − q̇_i) + k_g·τ_g,i(q) + k_f·τ_f,i(q̇)
//   其中 τ_ff = τ_g + τ_f 就是 tff（前馈力矩）
//
// 指令层与模式（2026-11-22 Phase 1，规划见 docs/changes/2026-11-22-01 §5.4.1）：
//   q_cmd = 上层最新目标（**只用于"到没到"的判定**）
//   q_up  = q_cmd 经 target_slew 限斜率后的参考（**只决定"怎么走"**）
//   q_ref = 本周期参考：未 done → q_up；done → lpf(q)。τ 用 q_ref 算。
//   done  = **整组**级锁存标记：
//             · 任一关节目标变化超过 target_deadband → done=false（整组重新索权）
//             · 全部关节连续 settle_sec 都在各自误差带内 → done=true（交给手动）
//   刚度：未 done 时趋向 kp_track/kd_track；done 后趋向 kp_free/kd_free，
//         且按 1/kp_up_sec、1/kp_down_sec 的**线性速率**过渡（不是一阶滤波，曲线可验收）。
//   为什么不用"误差>阈值"切模式：那样人一推就被误判成"上层要大动作" → 刚度升到 60 顶人。
//
// 两条分层边界（2026-11-22 起，见 docs/changes/2026-11-22-01-per-joint-controller-...）：
//   ① 一个实例只控 `joints` 里的 N 个关节（N 可以是 1，也可以是 7，含夹爪）；
//      重力前馈需要**整条运动链**的位形，所以另用 `chain_joints` 声明 6 个只读 position 接口。
//   ② 本控制器只写 effort 命令接口；硬件插件在 mit_torque 模式下把 effort 原样变成电机的
//      MIT 力矩帧（kp=kd=0），并按 `effort_scale` 换算成电机力矩单位。
//
//   为什么要能"只控一个关节"：把某个关节换成实验用的控制器（例如重力补偿实验）时，
//   其余关节必须继续跑**验证过的标准参数**。那时两个实例各自 claim 不同关节的 effort 接口，
//   ros2_control 允许它们同时 active（命令接口不重叠）。

#pragma once

#include <array>
#include <atomic>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "b601_mit_controller/b601_chain.hpp"

namespace b601_mit_controller
{

namespace rt = realtime_tools;

class B601MitController : public controller_interface::ControllerInterface
{
public:
  B601MitController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum class Mode { kHold, kMove };

  /// 用链常量 + 辨识参数算重力力矩（Newton-Euler 的静态项，见 b601_chain.hpp 注释）。
  /// 输入是**整条链**的 6 个关节角，输出是 6 个关节各自的保持力矩。
  void compute_gravity(
    const std::array<double, chain::kNumJoints> & q,
    std::array<double, chain::kNumJoints> & tau) const;

  void target_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & params);

  /// 逐关节参数：允许写一个标量（全体同值）或 N 元数组（逐关节）
  std::vector<double> to_per_joint(
    const std::string & name, const rclcpp::Parameter & p, std::size_t n) const;
  bool read_gain_params();
  /// 声明的 chain_joints 是否与 b601_chain.hpp 的整链一致（只允许 6 个、名字对得上）
  bool validate_chain_names();

  // ---- 配置 ----
  std::vector<std::string> joint_names_;     // 本实例控制的 N 个关节
  std::vector<std::string> chain_names_;     // 重力前馈所需整链（6 个，只读 position）
  std::array<double, 4 * chain::kNumGravityLinks> gravity_params_{};
  std::vector<double> friction_;             // N：库仑摩擦（关节单位力矩）
  // 运行时可调（ros2 param set 立即生效），因此用 atomic 避免与实时线程竞争。
  // 首调安全流程就是靠它：先把 gravity_scale 设 0，确认稳定再逐级升到 1。
  std::atomic<double> gravity_scale_{1.0};
  std::atomic<double> friction_scale_{0.0};   // 默认 0：先只上重力补偿
  // 逐关节增益（不是标量）：各关节等效惯量差 40 倍，用统一 kd 会让最轻的 joint6
  // 离散发散（MuJoCo 实测：joint6 的 I=0.0106 kg·m²，5 ms 周期下 kd 上限 4.22）。
  // 默认值按临界阻尼 kd=2√(kp·I) 给出，见 kDefaultKd*。
  // kp 也逐关节：真机位置反馈有 ~12 ms 延迟，迟滞弹簧的稳定判据是 ωn·T < ~0.5，
  // 而 ωn=√(kp/I)：j5/j6 用 kp=60 时 ωn 达 53/75 rad/s，ωn·T≈0.6–0.9 → 一动就自激
  // （2026-11-21 真机：j5 5.2 rad/s、j6 12 rad/s，连 kd=0 的纯弹簧都会振）。
  // 所以 j4–j6 用低 kp（≈20/10/8），把带宽压到 ωn·T≈0.3；它们的重力矩本来就小。
  // 夹爪是 prismatic（单位 m / N），量级完全不同，由它自己的 profile 给。
  std::vector<double> kp_hold_;        // 兼容保留：未给 kp_free 时作为 free 档的回退值
  std::vector<double> kd_hold_;
  // ---- 指令层（Phase 1）----
  std::vector<double> kp_free_;        // 等待（无权威）档刚度
  std::vector<double> kd_free_;
  std::vector<bool> free_enabled_;     // 该关节是否参与等待模式（夹爪也参与，决策⑭）
  std::vector<double> target_deadband_;   // 视为「同一指令」的门限（手臂 0.001 rad / 夹爪 1e-4 m）
  std::vector<double> settle_band_min_;
  std::vector<double> settle_band_max_;
  std::vector<double> settle_band_;    // 每周期按本次指令位移算出来的实际误差带
  double settle_band_pct_{0.05};
  double settle_sec_{3.0};
  double target_slew_{0.5};            // 关节单位/s，目标斜率上限
  double kp_up_sec_{0.8};
  double kp_down_sec_{2.0};
  double q_ref_lpf_hz_{30.0};
  double track_timeout_sec_{0.0};      // 0 = 关闭（被挡场景前期不做，决策⑩）
  std::vector<double> kp_move_;
  std::vector<double> kd_move_;
  std::vector<double> torque_limit_;         // N：逐关节上限（必须 ≤ 该关节 URDF max_effort）
  double max_error_{0.45};                   // 关节单位，PD 误差饱和（防状态异常时打巨力）
  std::atomic<double> move_threshold_{0.08};  // 关节单位
  std::atomic<double> hold_threshold_{0.03};  // 关节单位
  double vel_hold_threshold_{0.30};           // 关节单位/s
  double kp_ramp_sec_{0.35};
  double friction_vel_eps_{0.02};             // 关节单位/s，tanh 平滑速度
  bool use_gravity_{true};
  // 真机鲁棒性三件套（2026-11-21 加，见 docs/test-log.md 第七轮）：
  //  ① 速度一阶低通：达妙电机低速时速度反馈的量化抖动约 ±0.02 rad/s，
  //     一旦被激发会放大到 ±3 rad/s，kd 会把它直接放大成满力矩抖振；
  //  ② kd 项速度钳位：即使反馈瞬时跳变，阻尼力矩也有上限（保护轻关节 j4–j6）；
  //  ③ 力矩斜坡：单周期力矩变化受限，抖振不可能一步冲到限幅值。
  // 三者都只改变"输入质量/输出速率"，不改变 PD 控制律本身。
  std::atomic<double> velocity_lpf_hz_{12.0};
  std::atomic<double> max_vel_feedback_{2.0};
  std::atomic<double> torque_slew_{400.0};   // 关节单位/s，0 = 关闭
  // 电机侧阻抗（Phase 3）：位置环交给驱动器。控制器写 position/effort/kp/kd 四个命令接口，
  // effort 只带**前馈**（重力+摩擦）。目的：摆脱 200 Hz 控制周期与反馈延迟对刚度上限的限制。
  bool motor_impedance_{false};

  // ---- 运行时 ----
  // 状态/命令接口的**按名映射**索引（-1 = 未找到）。
  // 为什么不能再用 state_interfaces_[2*i]：非 mock 后端每关节导出 3 个状态接口
  // （position/velocity/effort），硬编码 2×i 会读到别的关节的值 ——
  // 2026-11-21 实测（MuJoCo 后端）：读到的 q 与真实位形完全错位，
  // PD 用错的值算出满力矩，170 ms 就把手臂推到限位。这也很可能是 2026-11-20
  // 真机"抽风"的成因之一。
  std::vector<int> pos_idx_;                 // N：本实例关节的 position
  std::vector<int> vel_idx_;                 // N：本实例关节的 velocity
  std::vector<int> cmd_idx_;                 // N：本实例关节的 effort 命令接口
  std::vector<int> pos_cmd_idx_;             // N：position 命令接口（仅 motor_impedance）
  std::vector<int> kp_idx_;                  // N：kp 命令接口（仅 motor_impedance）
  std::vector<int> kd_idx_;                  // N：kd 命令接口（仅 motor_impedance）
  std::array<int, chain::kNumJoints> chain_pos_idx_{-1, -1, -1, -1, -1, -1};
  std::vector<int> gravity_row_;             // N：本实例第 i 关节在整链里的行号（-1 = 不参与）
  bool bind_interfaces();

  std::array<double, chain::kNumJoints> chain_q_{};   // 整链位形（重力前馈用）
  // ---- 指令层运行时状态 ----
  std::vector<double> q_cmd_;      // 上层目标（判定基准）
  std::vector<double> q_up_;       // 限斜率后的参考（运动基准）
  std::vector<double> q_ref_;      // 本周期实际参考
  double t_in_band_{0.0};          // 整组连续在带内的累计时间
  bool done_{true};                // 整组执行标记（激活时先按"已到位"处理：不许有动作）
  uint64_t seq_seen_{0};           // 已处理到的指令序号
  double t_track_{0.0};            // 本段 TRACK 已持续的时间（给 track_timeout 用）
  std::vector<double> q_, qd_, qd_f_;   // qd_f_ = 低通后的速度（只给 kd 项用）
  std::vector<double> q_des_;
  std::vector<double> tau_cmd_;
  std::vector<double> tau_prev_;        // 上一周期实际下发力矩（力矩斜坡基准）
  std::array<double, chain::kNumJoints> tau_g_{};
  std::vector<double> kp_cur_, kd_cur_; // 实际生效值（含平滑过渡）
  Mode mode_{Mode::kHold};
  bool have_target_{false};
  std::atomic<bool> target_received_{false};   // 是否真的收到过话题目标
  std::atomic<uint64_t> target_seq_{0};        // 上层每来一帧有效目标就 +1
  bool have_state_{false};

  /// 指令：目标值 + 单调序号（必须同一个结构体一起交换，否则 RT 侧会读到"新目标+旧序号"）
  struct TargetCommand
  {
    std::vector<double> q;
    uint64_t seq{0};
  };
  rt::RealtimeBuffer<TargetCommand> target_buffer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  std::size_t debug_counter_{0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace b601_mit_controller
