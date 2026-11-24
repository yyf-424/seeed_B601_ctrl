#include "b601_mit_controller/b601_mit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <controller_interface/helpers.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace b601_mit_controller
{
namespace
{
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<double, 9>;   // 行主序

constexpr double kGravity = 9.81;

Vec3 mul(const Mat3 & R, const Vec3 & v)
{
  return {
    R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
    R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
    R[6] * v[0] + R[7] * v[1] + R[8] * v[2]};
}

Mat3 mul(const Mat3 & A, const Mat3 & B)
{
  Mat3 C{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      C[3 * i + j] = A[3 * i + 0] * B[0 + j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j];
    }
  }
  return C;
}

Vec3 add(const Vec3 & a, const Vec3 & b) {return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};}
Vec3 sub(const Vec3 & a, const Vec3 & b) {return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};}
Vec3 cross(const Vec3 & a, const Vec3 & b)
{
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
double dot(const Vec3 & a, const Vec3 & b) {return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];}

Mat3 from_array(const double * v)
{
  Mat3 R{};
  for (int i = 0; i < 9; ++i) {
    R[i] = v[i];
  }
  return R;
}

Vec3 from_array3(const double * v) {return {v[0], v[1], v[2]};}

/// Rodrigues：绕单位轴旋转 angle
Mat3 rot_axis(const Vec3 & axis, double angle)
{
  const double c = std::cos(angle), s = std::sin(angle), t = 1.0 - c;
  const double x = axis[0], y = axis[1], z = axis[2];
  return {
    t * x * x + c, t * x * y - s * z, t * x * z + s * y,
    t * x * y + s * z, t * y * y + c, t * y * z - s * x,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c};
}

/// 把参数读成"每个关节一个值"：接受标量（全体同值）或 N 元数组。
bool param_to_doubles(const rclcpp::Parameter & p, std::vector<double> & out)
{
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    out.assign(1, p.as_double());
    return true;
  }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    out = p.as_double_array();
    return true;
  }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    out.assign(1, static_cast<double>(p.as_int()));
    return true;
  }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    out.clear();
    for (const auto v : p.as_integer_array()) {
      out.push_back(static_cast<double>(v));
    }
    return true;
  }
  return false;
}

/// 线性推进到目标（不越过）。刚度斜坡、目标斜坡都用它 —— 相比一阶滤波，
/// 斜率是常数、到位时间可算，验收时能直接对表。
double move_toward(double cur, double target, double max_step)
{
  if (max_step <= 0.0) {
    return target;
  }
  const double d = target - cur;
  if (std::abs(d) <= max_step) {
    return target;
  }
  return cur + std::copysign(max_step, d);
}

}  // namespace

// ------------------------------------------------------------------ 接口声明
controller_interface::InterfaceConfiguration
B601MitController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : joint_names_) {
    if (motor_impedance_) {
      // 电机侧阻抗：位置环在驱动器里，控制器把 (q_ref, kp, kd) 一起下发
      cfg.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
      cfg.names.push_back(name + "/kp");
      cfg.names.push_back(name + "/kd");
    }
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return cfg;
}

controller_interface::InterfaceConfiguration B601MitController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // 本实例的关节：position + velocity
  for (const auto & name : joint_names_) {
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  // 整链位形：本实例没控的链关节也要读 position（重力前馈要用整条链）
  for (const auto & name : chain_names_) {
    if (std::find(joint_names_.begin(), joint_names_.end(), name) != joint_names_.end()) {
      continue;   // 已经声明过，别重复
    }
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return cfg;
}

// ---------------------------------------------------------------------- 初始化
std::vector<double> B601MitController::to_per_joint(
  const std::string & name, const rclcpp::Parameter & p, std::size_t n) const
{
  std::vector<double> raw;
  if (!param_to_doubles(p, raw)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "%s 的类型不支持（需要 double / double[]），当前 %s",
      name.c_str(), p.get_type_name().c_str());
    return {};
  }
  std::vector<double> out;
  if (raw.size() == n) {
    out = raw;
  } else if (raw.size() == 1) {
    out.assign(n, raw[0]);
  } else {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "%s 需要 1 个或 %zu 个值（= joints 的个数），实际 %zu", name.c_str(), n, raw.size());
    return {};
  }
  for (auto & v : out) {
    if (!std::isfinite(v)) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s 含非有限值", name.c_str());
      return {};
    }
  }
  return out;
}

bool B601MitController::validate_chain_names()
{
  if (chain_names_.size() != chain::kNumJoints) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "chain_joints 必须是 %zu 个（整条运动链），实际 %zu —— 重力前馈无法计算",
      chain::kNumJoints, chain_names_.size());
    return false;
  }
  for (std::size_t k = 0; k < chain::kNumJoints; ++k) {
    const std::string expect = chain::kJoints[k].name;
    if (chain_names_[k] != expect) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "chain_joints[%zu] 应为 %s（顺序必须与 b601_chain.hpp 一致），实际 %s",
        k, expect.c_str(), chain_names_[k].c_str());
      return false;
    }
  }
  return true;
}

bool B601MitController::read_gain_params()
{
  auto node = get_node();
  const std::size_t n = joint_names_.size();
  auto ensure = [&node](const std::string & name, auto default_value) {
      if (!node->has_parameter(name)) {
        node->declare_parameter(name, default_value);
      }
    };

  // 默认值 = 2026-11-21 真机验证过的那一套（只在 6 关节实例下有意义；
  // 其它关节数的实例必须由 profile 显式给值，否则下面 to_per_joint 会报错）
  ensure("kp_hold", std::vector<double>{6.0, 6.0, 6.0, 2.0, 1.0, 0.8});
  ensure("kd_hold", std::vector<double>{1.63, 3.15, 2.67, 0.37, 0.21, 0.13});
  ensure("kp_move", std::vector<double>{60.0, 60.0, 60.0, 20.0, 10.0, 8.0});
  ensure("kd_move", std::vector<double>{5.17, 9.97, 8.45, 1.17, 0.55, 0.35});
  ensure("friction", std::vector<double>(6, 0.0));
  ensure("torque_limit", 20.0);
  // 等待档：没给就回落到 hold 档（老配置仍能跑）
  ensure("kp_free", std::vector<double>(kp_hold_.begin(), kp_hold_.end()));
  ensure("kd_free", std::vector<double>(kd_hold_.begin(), kd_hold_.end()));
  ensure("free_enabled", std::vector<bool>(n, true));
  ensure("target_deadband", 0.001);
  ensure("settle_band_min", 0.01);
  ensure("settle_band_max", 0.05);

  kp_hold_ = to_per_joint("kp_hold", node->get_parameter("kp_hold"), n);
  // kp_free/kd_free 可能来自 hold 档的默认值，需要在 kp_hold_ 读完之后再取
  kp_free_ = to_per_joint("kp_free", node->get_parameter("kp_free"), n);
  kd_free_ = to_per_joint("kd_free", node->get_parameter("kd_free"), n);
  target_deadband_ = to_per_joint("target_deadband", node->get_parameter("target_deadband"), n);
  settle_band_min_ = to_per_joint("settle_band_min", node->get_parameter("settle_band_min"), n);
  settle_band_max_ = to_per_joint("settle_band_max", node->get_parameter("settle_band_max"), n);
  {
    std::vector<bool> raw;
    const auto & p = node->get_parameter("free_enabled");
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL_ARRAY) {
      raw = p.as_bool_array();
    } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      raw.assign(1, p.as_bool());
    } else {
      RCLCPP_ERROR(node->get_logger(), "free_enabled 类型不支持（需要 bool / bool[]）");
      return false;
    }
    free_enabled_.assign(n, true);
    if (raw.size() == n) {
      for (std::size_t i = 0; i < n; ++i) free_enabled_[i] = raw[i];
    } else if (raw.size() != 1) {
      RCLCPP_ERROR(node->get_logger(), "free_enabled 需要 1 或 %zu 个值，实际 %zu", n, raw.size());
      return false;
    } else {
      for (std::size_t i = 0; i < n; ++i) free_enabled_[i] = raw[0];
    }
  }
  kd_hold_ = to_per_joint("kd_hold", node->get_parameter("kd_hold"), n);
  kp_move_ = to_per_joint("kp_move", node->get_parameter("kp_move"), n);
  kd_move_ = to_per_joint("kd_move", node->get_parameter("kd_move"), n);
  friction_ = to_per_joint("friction", node->get_parameter("friction"), n);
  torque_limit_ = to_per_joint("torque_limit", node->get_parameter("torque_limit"), n);
  if (kp_hold_.size() != n || kd_hold_.size() != n || kp_move_.size() != n ||
    kd_move_.size() != n || friction_.size() != n || torque_limit_.size() != n ||
    kp_free_.size() != n || kd_free_.size() != n || target_deadband_.size() != n ||
    settle_band_min_.size() != n || settle_band_max_.size() != n)
  {
    return false;
  }
  for (std::size_t i = 0; i < n; ++i) {
    kp_free_[i] = std::max(0.0, kp_free_[i]);
    kd_free_[i] = std::max(0.0, kd_free_[i]);
    target_deadband_[i] = std::max(0.0, target_deadband_[i]);
    settle_band_min_[i] = std::max(0.0, settle_band_min_[i]);
    settle_band_max_[i] = std::max(settle_band_min_[i], settle_band_max_[i]);
    settle_band_.assign(n, settle_band_max_[i]);
  }
  for (std::size_t i = 0; i < n; ++i) {
    kp_hold_[i] = std::max(0.0, kp_hold_[i]);
    kd_hold_[i] = std::max(0.0, kd_hold_[i]);
    kp_move_[i] = std::max(0.0, kp_move_[i]);
    kd_move_[i] = std::max(0.0, kd_move_[i]);
    torque_limit_[i] = std::max(0.0, torque_limit_[i]);
  }
  return true;
}

controller_interface::CallbackReturn B601MitController::on_init()
{
  // 注意：controller_manager 可能多次调用 on_init（例如先加载后重新配置），
  // 所以每个参数都要"没声明过才声明"，否则会抛 'parameter has already been declared'。
  try {
    auto node = get_node();
    auto ensure = [&node](const std::string & name, auto default_value) {
        if (!node->has_parameter(name)) {
          node->declare_parameter(name, default_value);
        }
      };

    // ---- 关节集合：本实例控哪些 + 整链有哪些（后者只读 position）----
    ensure("joints", std::vector<std::string>{
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    joint_names_ = node->get_parameter("joints").as_string_array();
    ensure("chain_joints", std::vector<std::string>{
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    chain_names_ = node->get_parameter("chain_joints").as_string_array();
    if (joint_names_.empty()) {
      RCLCPP_ERROR(node->get_logger(), "joints 不能为空");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (!validate_chain_names()) {
      return controller_interface::CallbackReturn::ERROR;
    }

    ensure("gravity_params", std::vector<double>(4 * chain::kNumGravityLinks, 0.0));
    const auto gp = node->get_parameter("gravity_params").as_double_array();
    if (gp.size() != 4 * chain::kNumGravityLinks) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "gravity_params 长度应为 %zu，实际 %zu",
        4 * chain::kNumGravityLinks, gp.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    std::copy(gp.begin(), gp.end(), gravity_params_.begin());

    if (!read_gain_params()) {
      return controller_interface::CallbackReturn::ERROR;
    }

    ensure("gravity_scale", gravity_scale_.load());
    gravity_scale_ = std::min(1.5, std::max(0.0, node->get_parameter("gravity_scale").as_double()));
    ensure("friction_scale", friction_scale_.load());
    friction_scale_ = std::min(2.0, std::max(0.0, node->get_parameter("friction_scale").as_double()));
    ensure("max_error", max_error_);
    max_error_ = std::max(1e-3, node->get_parameter("max_error").as_double());
    // 旧的"误差阈值调度"参数（move/hold_threshold、vel_hold_threshold、kp_ramp_sec）
    // 在 Phase 1 已**不再用于决定模式**：模式只由「指令变化 + 整组到位」决定。
    // 仍然声明，是为了让旧 yaml/工具不报"未声明参数"；改它们不会有任何效果。
    ensure("move_threshold", move_threshold_.load());
    move_threshold_ = std::max(0.0, node->get_parameter("move_threshold").as_double());
    ensure("hold_threshold", hold_threshold_.load());
    hold_threshold_ = std::max(0.0, node->get_parameter("hold_threshold").as_double());
    ensure("vel_hold_threshold", vel_hold_threshold_);
    vel_hold_threshold_ = std::max(0.0, node->get_parameter("vel_hold_threshold").as_double());
    ensure("kp_ramp_sec", kp_ramp_sec_);
    kp_ramp_sec_ = std::max(1e-3, node->get_parameter("kp_ramp_sec").as_double());
    ensure("friction_vel_eps", friction_vel_eps_);
    friction_vel_eps_ = std::max(1e-4, node->get_parameter("friction_vel_eps").as_double());
    ensure("motor_impedance", motor_impedance_);
    motor_impedance_ = node->get_parameter("motor_impedance").as_bool();
    ensure("use_gravity", use_gravity_);
    use_gravity_ = node->get_parameter("use_gravity").as_bool();
    // ---- 真机鲁棒性三件套（默认值即为保守值，可用 ros2 param set 现场调）----
    // ---- 指令层（Phase 1）----
    ensure("settle_band_pct", settle_band_pct_);
    settle_band_pct_ = std::clamp(node->get_parameter("settle_band_pct").as_double(), 0.0, 1.0);
    ensure("settle_sec", settle_sec_);
    settle_sec_ = std::max(0.05, node->get_parameter("settle_sec").as_double());
    ensure("target_slew", target_slew_);
    target_slew_ = std::max(1e-3, node->get_parameter("target_slew").as_double());
    ensure("kp_up_sec", kp_up_sec_);
    kp_up_sec_ = std::max(1e-3, node->get_parameter("kp_up_sec").as_double());
    ensure("kp_down_sec", kp_down_sec_);
    kp_down_sec_ = std::max(1e-3, node->get_parameter("kp_down_sec").as_double());
    ensure("q_ref_lpf_hz", q_ref_lpf_hz_);
    q_ref_lpf_hz_ = std::max(0.5, node->get_parameter("q_ref_lpf_hz").as_double());
    ensure("track_timeout_sec", track_timeout_sec_);
    track_timeout_sec_ = std::max(0.0, node->get_parameter("track_timeout_sec").as_double());
    ensure("velocity_lpf_hz", velocity_lpf_hz_.load());
    velocity_lpf_hz_ = std::max(0.5, node->get_parameter("velocity_lpf_hz").as_double());
    ensure("max_vel_feedback", max_vel_feedback_.load());
    max_vel_feedback_ = std::max(0.01, node->get_parameter("max_vel_feedback").as_double());
    ensure("torque_slew", torque_slew_.load());
    torque_slew_ = std::max(0.0, node->get_parameter("torque_slew").as_double());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "参数读取失败：%s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "MIT 控制器：%zu 个关节（%s），整链重力前馈用 %zu 个关节的位形",
    joint_names_.size(), joint_names_.front().c_str(), chain_names_.size());
  {
    std::string gains;
    char buf[64];
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      std::snprintf(
        buf, sizeof(buf), "%s%s kp=%.1f/%.1f kd=%.2f/%.2f", i ? " |" : "",
        joint_names_[i].c_str(), kp_hold_[i], kp_move_[i], kd_hold_[i], kd_move_[i]);
      gains += buf;
    }
    RCLCPP_INFO(get_node()->get_logger(), "逐关节增益：%s", gains.c_str());
  }
  RCLCPP_INFO(
    get_node()->get_logger(),
    "指令层：slew=%.2f 死区=%.4f 误差带=%.0f%%×位移且夹在[%.4f,%.4f] 到位判定=%.2fs "
    "刚上升/下降=%.2f/%.2fs 等待档某关节禁用=%s track_timeout=%.1fs",
    target_slew_, target_deadband_.front(), settle_band_pct_ * 100.0,
    settle_band_min_.front(), settle_band_max_.front(), settle_sec_,
    kp_up_sec_, kp_down_sec_,
    std::any_of(free_enabled_.begin(), free_enabled_.end(), [](bool b) {return !b;}) ? "是" : "否",
    track_timeout_sec_);
  RCLCPP_INFO(
    get_node()->get_logger(),
    "守卫：重力前馈 %.2f 摩擦 %.2f 速度低通 %.1f Hz / kd 钳位 %.2f / 力矩斜坡 %.0f",
    gravity_scale_.load(), friction_scale_.load(),
    velocity_lpf_hz_.load(), max_vel_feedback_.load(), torque_slew_.load());
  return controller_interface::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult B601MitController::parameter_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  const std::size_t n = joint_names_.size();
  for (const auto & p : params) {
    const auto & name = p.get_name();
    if (name == "gravity_scale") {
      gravity_scale_ = std::clamp(p.as_double(), 0.0, 1.5);
    } else if (name == "friction_scale") {
      friction_scale_ = std::clamp(p.as_double(), 0.0, 2.0);
    } else if (name == "kp_hold" || name == "kd_hold" || name == "kp_move" || name == "kd_move" ||
      name == "friction" || name == "torque_limit")
    {
      auto v = to_per_joint(name, p, n);
      if (v.size() != n) {
        result.successful = false;
        result.reason = name + " 长度不对（需要 1 或 " + std::to_string(n) + " 个值）";
        return result;
      }
      for (auto & x : v) {
        x = std::max(0.0, x);
      }
      if (name == "kp_hold") {
        kp_hold_ = v;
      } else if (name == "kd_hold") {
        kd_hold_ = v;
      } else if (name == "kp_move") {
        kp_move_ = v;
      } else if (name == "kd_move") {
        kd_move_ = v;
      } else if (name == "friction") {
        friction_ = v;
      } else {
        torque_limit_ = v;
      }
    } else if (name == "move_threshold") {
      move_threshold_ = std::max(0.0, p.as_double());
    } else if (name == "hold_threshold") {
      hold_threshold_ = std::max(0.0, p.as_double());
    } else if (name == "max_error") {
      max_error_ = std::max(1e-3, p.as_double());
    } else if (name == "velocity_lpf_hz") {
      velocity_lpf_hz_ = std::max(0.5, p.as_double());
    } else if (name == "max_vel_feedback") {
      max_vel_feedback_ = std::max(0.01, p.as_double());
    } else if (name == "torque_slew") {
      torque_slew_ = std::max(0.0, p.as_double());
    } else {
      continue;
    }
    RCLCPP_INFO(get_node()->get_logger(), "运行时更新 %s", name.c_str());
  }
  return result;
}

bool B601MitController::bind_interfaces()
{
  const std::size_t n = joint_names_.size();
  auto joint_index = [this](const std::string & name) -> int {
      for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        if (joint_names_[i] == name) {
          return static_cast<int>(i);
        }
      }
      return -1;
    };

  pos_idx_.assign(n, -1);
  vel_idx_.assign(n, -1);
  cmd_idx_.assign(n, -1);
  pos_cmd_idx_.assign(n, -1);
  kp_idx_.assign(n, -1);
  kd_idx_.assign(n, -1);
  chain_pos_idx_.fill(-1);

  for (std::size_t k = 0; k < state_interfaces_.size(); ++k) {
    const auto & si = state_interfaces_[k];
    // Humble 的 get_name() 返回 "<关节>/<接口>"，关节名要用 get_prefix_name()
    const std::string prefix = si.get_prefix_name();
    const std::string iface = si.get_interface_name();
    if (iface == hardware_interface::HW_IF_POSITION) {
      const int j = joint_index(prefix);
      if (j >= 0) {
        pos_idx_[j] = static_cast<int>(k);
      }
      // 整链位形：无论本实例是否控这个关节，都要能读到
      for (std::size_t c = 0; c < chain_names_.size(); ++c) {
        if (chain_names_[c] == prefix) {
          chain_pos_idx_[c] = static_cast<int>(k);
        }
      }
    } else if (iface == hardware_interface::HW_IF_VELOCITY) {
      const int j = joint_index(prefix);
      if (j >= 0) {
        vel_idx_[j] = static_cast<int>(k);
      }
    }
  }
  for (std::size_t k = 0; k < command_interfaces_.size(); ++k) {
    const int j = joint_index(command_interfaces_[k].get_prefix_name());
    if (j < 0) {
      continue;
    }
    const auto & iface = command_interfaces_[k].get_interface_name();
    if (iface == hardware_interface::HW_IF_EFFORT && cmd_idx_[j] < 0) {
      cmd_idx_[j] = static_cast<int>(k);
    } else if (iface == hardware_interface::HW_IF_POSITION && pos_cmd_idx_[j] < 0) {
      pos_cmd_idx_[j] = static_cast<int>(k);
    } else if (iface == "kp" && kp_idx_[j] < 0) {
      kp_idx_[j] = static_cast<int>(k);
    } else if (iface == "kd" && kd_idx_[j] < 0) {
      kd_idx_[j] = static_cast<int>(k);
    }
  }
  if (motor_impedance_) {
    for (std::size_t i = 0; i < n; ++i) {
      if (pos_cmd_idx_[i] < 0 || kp_idx_[i] < 0 || kd_idx_[i] < 0) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "motor_impedance=true 但 %s 缺 position/kp/kd 命令接口 —— "
          "URDF 里需要 dm_command_mode:=mit_impedance", joint_names_[i].c_str());
        return false;
      }
    }
  }

  // 本实例关节 → 整链行号（夹爪等不在链里的关节记 -1：它的重力项按 0 处理）
  gravity_row_.assign(n, -1);
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t c = 0; c < chain::kNumJoints; ++c) {
      if (joint_names_[i] == std::string(chain::kJoints[c].name)) {
        gravity_row_[i] = static_cast<int>(c);
        break;
      }
    }
  }

  for (std::size_t i = 0; i < n; ++i) {
    if (pos_idx_[i] < 0 || vel_idx_[i] < 0 || cmd_idx_[i] < 0) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "接口绑定失败：%s 缺 position/velocity 状态接口或命令接口（pos=%d vel=%d cmd=%d）",
        joint_names_[i].c_str(), pos_idx_[i], vel_idx_[i], cmd_idx_[i]);
      return false;
    }
  }
  for (std::size_t c = 0; c < chain_names_.size(); ++c) {
    if (chain_pos_idx_[c] < 0) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "整链位形读不到：chain_joints[%zu]=%s 没有 position 状态接口",
        c, chain_names_[c].c_str());
      return false;
    }
  }
  RCLCPP_INFO(
    get_node()->get_logger(),
    "接口已按名绑定：%zu 状态 / %zu 命令接口（本实例 %zu 关节，整链 %zu 关节）",
    state_interfaces_.size(), command_interfaces_.size(), n, chain_names_.size());
  return true;
}

controller_interface::CallbackReturn B601MitController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  target_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
    "~/joint_target", rclcpp::SystemDefaultsQoS(),
    std::bind(&B601MitController::target_callback, this, std::placeholders::_1));
  debug_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("~/debug", 10);

  param_cb_handle_ = node->add_on_set_parameters_callback(
    std::bind(&B601MitController::parameter_callback, this, std::placeholders::_1));

  state_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("~/state", 10);

  const std::size_t n = joint_names_.size();
  TargetCommand init;
  init.q.assign(n, 0.0);
  init.seq = 0;
  target_buffer_.writeFromNonRT(init);
  have_target_ = false;
  have_state_ = false;
  done_ = true;                 // 激活后先按"已到位"：不许有动作，等人发目标
  t_in_band_ = 0.0;
  t_track_ = 0.0;
  seq_seen_ = 0;
  RCLCPP_INFO(node->get_logger(), "已配置：等待目标话题 %s/joint_target", node->get_name());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn B601MitController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!bind_interfaces()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  const std::size_t n = joint_names_.size();
  q_cmd_.assign(n, 0.0);
  q_up_.assign(n, 0.0);
  q_ref_.assign(n, 0.0);
  settle_band_.assign(n, 0.0);
  q_.assign(n, 0.0);
  qd_.assign(n, 0.0);
  qd_f_.assign(n, 0.0);
  q_des_.assign(n, 0.0);
  tau_cmd_.assign(n, 0.0);
  tau_prev_.assign(n, 0.0);
  kp_cur_.assign(n, 0.0);
  kd_cur_.assign(n, 0.0);
  tau_g_.fill(0.0);

  // 从状态接口读当前位姿，作为初始目标 —— 激活瞬间不许有动作
  for (std::size_t i = 0; i < n; ++i) {
    q_[i] = state_interfaces_[pos_idx_[i]].get_value();
    qd_[i] = state_interfaces_[vel_idx_[i]].get_value();
    qd_f_[i] = qd_[i];        // 低通状态从实测速度起步，避免首周期突跳
    q_des_[i] = q_[i];
    q_cmd_[i] = q_[i];
    q_up_[i] = q_[i];
    q_ref_[i] = q_[i];
    settle_band_[i] = settle_band_max_[i];
    tau_cmd_[i] = 0.0;
    tau_prev_[i] = 0.0;
    // 激活时刚度必须是"当前状态该有的那个"：
    //   free_enabled=true（等待模式）→ kp_free（软，可手推，符合设计）
    //   free_enabled=false（纯 PD 对标 pos_vel）→ kp_move
    // ⚠ 以前无条件从 kp_free 起步，纯 PD 配置下会有 0.8 s 的"软手爬坡"，
    //   仿真里 joint2 在这 0.8 s 内掉到 6.26 rad/s 直接撞安全急停。
    kp_cur_[i] = free_enabled_[i] ? kp_free_[i] : kp_move_[i];
    kd_cur_[i] = free_enabled_[i] ? kd_free_[i] : kd_move_[i];
    command_interfaces_[cmd_idx_[i]].set_value(0.0);
    if (motor_impedance_) {
      command_interfaces_[pos_cmd_idx_[i]].set_value(q_[i]);
      command_interfaces_[kp_idx_[i]].set_value(0.0);   // 先零刚度，等第一周期再给
      command_interfaces_[kd_idx_[i]].set_value(0.0);
    }
  }
  for (std::size_t c = 0; c < chain_names_.size(); ++c) {
    chain_q_[c] = state_interfaces_[chain_pos_idx_[c]].get_value();
  }
  have_state_ = true;
  have_target_ = true;
  target_received_ = false;          // 话题目标还没来，先以当前位形为 q_cmd
  done_ = true;                      // 等待模式：没有权威、低刚度、可以手推
  t_in_band_ = 0.0;
  t_track_ = 0.0;
  {
    TargetCommand init;
    init.q = q_cmd_;
    init.seq = target_seq_.load();
    seq_seen_ = init.seq;
    target_buffer_.writeFromNonRT(init);
  }
  RCLCPP_INFO(
    get_node()->get_logger(),
    "已激活：%zu 关节，q_cmd=当前位形，进入**等待模式**（kp_free=%.2f，无权威）。"
    "重力补偿 %s，摩擦补偿 %s",
    n, kp_free_.front(), use_gravity_ ? "开" : "关", friction_scale_.load() > 1e-6 ? "开" : "关");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn B601MitController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & ci : command_interfaces_) {
    ci.set_value(0.0);      // 退出时把力矩命令（以及 kp/kd）清零
  }
  RCLCPP_INFO(get_node()->get_logger(), "已停用：力矩命令清零");
  return controller_interface::CallbackReturn::SUCCESS;
}

void B601MitController::target_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  const std::size_t n = joint_names_.size();
  std::vector<double> tgt(n, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    bool found = false;
    for (std::size_t k = 0; k < msg->name.size() && k < msg->position.size(); ++k) {
      if (msg->name[k] == joint_names_[i]) {
        tgt[i] = msg->position[k];
        found = true;
        break;
      }
    }
    if (!found) {
      // 消息里缺本实例的关节 → 整帧丢弃。宁愿不动，也不要"半帧生效"。
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "目标消息缺关节 %s —— 整帧丢弃（本实例需要 %zu 个关节名齐全）",
        joint_names_[i].c_str(), n);
      return;
    }
  }
  TargetCommand cmd;
  cmd.q = tgt;
  cmd.seq = ++target_seq_;      // 每帧有效目标一个序号；RT 侧据此判断"有没有新指令"
  target_buffer_.writeFromNonRT(cmd);
  target_received_ = true;
}

// ---------------------------------------------------------------- 重力前馈计算
void B601MitController::compute_gravity(
  const std::array<double, chain::kNumJoints> & q, std::array<double, chain::kNumJoints> & tau) const
{
  tau.fill(0.0);
  // ① 正运动学：6 个可动关节的 world 位置/姿态，以及每根轴的世界方向
  Vec3 p_child[chain::kNumJoints];
  Mat3 R_child[chain::kNumJoints];
  Vec3 axis_world[chain::kNumJoints];
  Vec3 p_prev{0.0, 0.0, 0.0};
  Mat3 R_prev{1, 0, 0, 0, 1, 0, 0, 0, 1};
  for (std::size_t i = 0; i < chain::kNumJoints; ++i) {
    const auto & j = chain::kJoints[i];
    const Vec3 xyz = from_array3(j.xyz);
    const Mat3 rot = from_array(j.rot);
    const Vec3 axis = from_array3(j.axis);
    const Vec3 p_joint = add(p_prev, mul(R_prev, xyz));
    const Mat3 R_joint = mul(R_prev, rot);
    axis_world[i] = mul(R_joint, axis);
    R_child[i] = mul(R_joint, rot_axis(axis, q[i]));
    p_child[i] = p_joint;
    p_prev = p_joint;
    R_prev = R_child[i];
  }

  // ② 逐 link 累加：τ_i += a_i · ( (m·p_com − m·p_joint_i) × g )
  const Vec3 g_vec{0.0, 0.0, -kGravity};
  for (std::size_t k = 0; k < chain::kNumGravityLinks; ++k) {
    const auto & link = chain::kGravityLinks[k];
    const double m = gravity_params_[4 * k];
    const Vec3 mu{
      gravity_params_[4 * k + 1], gravity_params_[4 * k + 2], gravity_params_[4 * k + 3]};
    if (std::abs(m) < 1e-9 && std::abs(mu[0]) + std::abs(mu[1]) + std::abs(mu[2]) < 1e-9) {
      continue;
    }
    const int d = link.joint_depth;
    const Vec3 p_link = add(p_child[d], mul(R_child[d], from_array3(link.xyz)));
    const Mat3 R_link = mul(R_child[d], from_array(link.rot));
    // m·p_com = m·p_link + R·(m·c)
    const Vec3 m_pcom = add(
      {m * p_link[0], m * p_link[1], m * p_link[2]}, mul(R_link, mu));
    for (int i = 0; i <= d; ++i) {
      const Vec3 m_pjoint{
        m * p_child[i][0], m * p_child[i][1], m * p_child[i][2]};
      tau[i] += dot(axis_world[i], cross(sub(m_pcom, m_pjoint), g_vec));
    }
  }
}

// ------------------------------------------------------------------ 主控制循环
controller_interface::return_type B601MitController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const std::size_t n = joint_names_.size();
  if (n == 0 || q_.size() != n) {
    return controller_interface::return_type::ERROR;
  }

  const double dt = std::clamp(period.seconds(), 1e-4, 0.05);

  for (std::size_t i = 0; i < n; ++i) {
    q_[i] = state_interfaces_[pos_idx_[i]].get_value();
    qd_[i] = state_interfaces_[vel_idx_[i]].get_value();
  }
  // 整链位形（本实例没控的关节也要读：重力前馈算的是整条链）
  for (std::size_t c = 0; c < chain_names_.size(); ++c) {
    chain_q_[c] = state_interfaces_[chain_pos_idx_[c]].get_value();
  }

  // ================== ① 指令接受（整组） ==================
  // 判据不是"误差多大"，而是"上层目标有没有变"。人推只改 q、不改 q_cmd，
  // 所以永远不会被误判成"上层要大动作"（这是 Phase 1 的核心修正）。
  const auto * cmd = target_buffer_.readFromNonRT();
  if (cmd != nullptr && have_target_ && target_received_.load() && cmd->q.size() == n &&
    cmd->seq != seq_seen_)
  {
    bool changed = false;
    for (std::size_t i = 0; i < n; ++i) {
      if (std::abs(cmd->q[i] - q_cmd_[i]) > target_deadband_[i]) {
        changed = true;
        break;
      }
    }
    if (changed) {
      // 误差带按"本次指令的位移"算，并夹在 [min,max]：
      //   太小 → 永远到不了（低增益下垂）；太大 → 大幅动作时提前宣布到位。
      for (std::size_t i = 0; i < n; ++i) {
        const double disp = std::abs(cmd->q[i] - q_cmd_[i]);
        settle_band_[i] = std::clamp(
          settle_band_pct_ * disp, settle_band_min_[i], settle_band_max_[i]);
        q_cmd_[i] = cmd->q[i];
      }
      done_ = false;          // 整组重新索权
      t_in_band_ = 0.0;
      t_track_ = 0.0;
      RCLCPP_INFO(
        get_node()->get_logger(), "新指令 → TRACK（带=%.4f，位移=%.4f）",
        settle_band_.front(), std::abs(q_cmd_.front() - q_ref_.front()));
    }
    seq_seen_ = cmd->seq;
  }

  // ================== ② 目标斜坡（只决定"怎么走"） ==================
  for (std::size_t i = 0; i < n; ++i) {
    q_up_[i] = move_toward(q_up_[i], q_cmd_[i], target_slew_ * dt);
  }

  // ---- 速度低通（只给 kd 项用）----
  // 真机实测：达妙电机在低速/低负载时速度反馈是量化+抖动的（LSB 约 0.007 rad/s），
  // 且安静时也会有 ±0.02 rad/s 的跳变；kd 乘上它会产出 ±0.07 N·m 的力矩噪声，
  // 一旦被激发（例如多条目标同时起步）就进入自激抖振，0.1 s 内把轻关节打到 ±10 N·m。
  // 一阶低通把这类高频抖动压掉，同时不改变低频下的刚度/阻尼特性。
  const double tau_v = 1.0 / (2.0 * M_PI * velocity_lpf_hz_.load());
  const double a_v = std::min(1.0, dt / std::max(tau_v, 1e-4));
  const double vfb_clamp = max_vel_feedback_.load();
  for (std::size_t i = 0; i < n; ++i) {
    qd_f_[i] += a_v * (qd_[i] - qd_f_[i]);
  }

  // ---- 重力前馈（整链算，取本实例关节对应的行）----
  if (use_gravity_) {
    compute_gravity(chain_q_, tau_g_);
  } else {
    tau_g_.fill(0.0);
  }

  // ================== ③ 整组到位判定（对着 q_cmd，不用斜坡数据） ==================
  if (!done_) {
    bool in_band = true;
    for (std::size_t i = 0; i < n; ++i) {
      // 允许带内小幅震荡：只看"在不在带里"，不判速度
      if (std::abs(q_cmd_[i] - q_[i]) >= settle_band_[i]) {
        in_band = false;
        break;
      }
    }
    t_in_band_ = in_band ? t_in_band_ + dt : 0.0;
    t_track_ += dt;
    if (t_in_band_ >= settle_sec_) {
      done_ = true;          // 锁存：只有"新指令"能清
      RCLCPP_INFO(get_node()->get_logger(), "到位（%.1f s 在带内）→ 进入等待模式（低刚度）",
        t_in_band_);
    } else if (track_timeout_sec_ > 0.0 && t_track_ > track_timeout_sec_) {
      done_ = true;
      RCLCPP_WARN(get_node()->get_logger(),
        "TRACK 超过 %.1f s 仍未到位 → 按「尽力完成」锁存（目标不可达？）", track_timeout_sec_);
    }
  }

  // ================== ④ 参考量 + 刚度目标 ==================
  const double a_ref = std::min(1.0, dt * 2.0 * M_PI * q_ref_lpf_hz_);
  for (std::size_t i = 0; i < n; ++i) {
    const bool authoritative = !done_ || !free_enabled_[i];
    if (authoritative) {
      q_ref_[i] = q_up_[i];                       // 上层有权威：跟斜坡走
    } else {
      q_ref_[i] += a_ref * (q_[i] - q_ref_[i]);   // 无权威：跟着实测（手推到哪停到哪）
    }
    q_des_[i] = q_ref_[i];    // 兼容旧字段（debug 里用过）
  }

  // ================== ⑤ 变刚度（线性速率斜坡） ==================
  const double ramp_sec = done_ ? kp_down_sec_ : kp_up_sec_;
  const double max_dtau = torque_slew_.load() * dt;   // 关节单位/周期；0 = 不限

  for (std::size_t i = 0; i < n; ++i) {
    const bool authoritative = !done_ || !free_enabled_[i];
    const double kp_tgt = authoritative ? kp_move_[i] : kp_free_[i];
    const double kd_tgt = authoritative ? kd_move_[i] : kd_free_[i];
    // 每关节各自用"该关节 free↔track 的跨度 / 斜坡时间"作为线性斜率
    const double kp_step = std::abs(kp_move_[i] - kp_free_[i]) / ramp_sec * dt;
    const double kd_step = std::abs(kd_move_[i] - kd_free_[i]) / ramp_sec * dt;
    kp_cur_[i] = move_toward(kp_cur_[i], kp_tgt, kp_step);
    kd_cur_[i] = move_toward(kd_cur_[i], kd_tgt, kd_step);

    // 误差饱和：状态反馈一旦异常（比如读数跳变），这里能拦住"巨大力矩"这种灾难
    const double err = std::clamp(q_ref_[i] - q_[i], -max_error_, max_error_);
    // kd 项速度钳位：正常斜坡（≤0.5 rad/s）根本碰不到它，
    // 只有反馈出现 2 rad/s 以上的跳变时才会削平，防止阻尼力矩变成"抖振发动机"。
    const double qd_used = std::clamp(qd_f_[i], -vfb_clamp, vfb_clamp);
    // 摩擦前馈：−f_c·tanh(q̇/eps)，用 tanh 而不是 sign 是为了在 0 速度附近平滑
    const double tau_f = -friction_[i] * std::tanh(qd_f_[i] / friction_vel_eps_);
    // 重力前馈：本实例关节若不在整链里（例如夹爪），重力项按 0（prismatic 轴向重力分量为 0）
    const double tau_grav = (gravity_row_[i] >= 0)
      ? tau_g_[static_cast<std::size_t>(gravity_row_[i])]
      : 0.0;

    double tau = kp_cur_[i] * err + kd_cur_[i] * (-qd_used)
      + gravity_scale_.load() * tau_grav + friction_scale_.load() * tau_f;

    // 力矩斜坡：限制单周期变化量，抖振不可能一步冲到限幅值
    if (max_dtau > 0.0) {
      tau = std::clamp(tau, tau_prev_[i] - max_dtau, tau_prev_[i] + max_dtau);
    }
    if (motor_impedance_) {
      // 电机侧阻抗：PD 由驱动器内部闭环（1 kHz 量级）。控制器只负责
      //   · position 命令 = 参考量（斜坡后的 q_ref）
      //   · kp/kd 命令   = 当前变刚度值（驱动器直接用）
      //   · effort 命令  = **只含前馈**（重力+摩擦），不再重复 PD
      const double ff = gravity_scale_.load() * tau_grav + friction_scale_.load() * tau_f;
      const double tlim = torque_limit_[i];
      const double ff_out = std::clamp(ff, -tlim, tlim);
      command_interfaces_[pos_cmd_idx_[i]].set_value(q_ref_[i]);
      command_interfaces_[kp_idx_[i]].set_value(kp_cur_[i]);
      command_interfaces_[kd_idx_[i]].set_value(kd_cur_[i]);
      command_interfaces_[cmd_idx_[i]].set_value(ff_out);
      tau_cmd_[i] = ff_out;
      tau_prev_[i] = ff_out;
      continue;
    }
    const double tlim = torque_limit_[i];
    tau = std::clamp(tau, -tlim, tlim);
    tau_cmd_[i] = tau;
    tau_prev_[i] = tau;
    command_interfaces_[cmd_idx_[i]].set_value(tau);
  }

  // ---- 低频调试输出（约 10 Hz）----
  ++debug_counter_;
  if (debug_counter_ % 20 == 0 && state_pub_) {
    std_msgs::msg::Float64MultiArray st;
    st.data.assign(n * 8 + 4, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
      const double tau_grav = (gravity_row_[i] >= 0)
        ? tau_g_[static_cast<std::size_t>(gravity_row_[i])] : 0.0;
      st.data[i] = tau_cmd_[i];
      st.data[n + i] = tau_grav;
      st.data[2 * n + i] = kp_cur_[i];
      st.data[3 * n + i] = kd_cur_[i];
      st.data[4 * n + i] = q_cmd_[i] - q_[i];      // 判定用的误差
      st.data[5 * n + i] = q_ref_[i] - q_[i];      // 控制用的误差
      st.data[6 * n + i] = settle_band_[i];
      st.data[7 * n + i] = q_up_[i] - q_cmd_[i];   // 斜坡残差
    }
    st.data[8 * n + 0] = done_ ? 1.0 : 0.0;
    st.data[8 * n + 1] = static_cast<double>(seq_seen_);
    st.data[8 * n + 2] = t_in_band_;
    st.data[8 * n + 3] = t_track_;
    state_pub_->publish(st);
  }
  if (debug_counter_ % 20 == 0 && debug_pub_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(n * 4);
    for (std::size_t i = 0; i < n; ++i) {
      const double tau_grav = (gravity_row_[i] >= 0)
        ? tau_g_[static_cast<std::size_t>(gravity_row_[i])]
        : 0.0;
      msg.data[i] = tau_cmd_[i];
      // 这里输出**未缩放**的 τ_g：它是模型量（与 compare_taug.py 的 Python 模型逐位对拍用），
      // 缩放系数 gravity_scale 是调参旋钮，不该混进模型量里。
      msg.data[n + i] = tau_grav;
      msg.data[2 * n + i] = kp_cur_[i];
      msg.data[3 * n + i] = q_des_[i] - q_[i];
    }
    debug_pub_->publish(msg);
  }
  return controller_interface::return_type::OK;
}

}  // namespace b601_mit_controller

PLUGINLIB_EXPORT_CLASS(
  b601_mit_controller::B601MitController, controller_interface::ControllerInterface)
