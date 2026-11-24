#include "b601_hardware/dm_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <thread>

#include "b601_hardware/motor_abi.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace b601_hardware
{
namespace
{
// 一个"接口名 → 指针"的公共小工具：只导出 URDF 里声明过的接口，
// 不硬编码 position/velocity，避免与 URDF 不一致时静默错配。
rclcpp::Logger logger()
{
  // 统一 logger 名，rqt_console / ros2 topic echo /rosout 里按它过滤
  return rclcpp::get_logger("DmSystemHardware");
}

std::string abi_error_text()
{
  const char * msg = motor_last_error_message();
  return msg == nullptr ? std::string("unknown motor_abi error") : std::string(msg);
}
}  // namespace

// ---------------------------------------------------------------- 参数小工具
double DmSystemHardware::get_double(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  double fallback, const std::string & where)
{
  const auto it = params.find(key);
  if (it == params.end() || it->second.empty()) {
    return fallback;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    RCLCPP_WARN(
      logger(),
      "%s 的参数 %s='%s' 不是合法数字，回落为 %f", where.c_str(), key.c_str(),
      it->second.c_str(), fallback);
    return fallback;
  }
}

bool DmSystemHardware::get_bool(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  bool fallback, const std::string & where)
{
  const auto it = params.find(key);
  if (it == params.end() || it->second.empty()) {
    return fallback;
  }
  std::string v = it->second;
  for (auto & ch : v) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  if (v == "true" || v == "1" || v == "yes" || v == "on") {
    return true;
  }
  if (v == "false" || v == "0" || v == "no" || v == "off") {
    return false;
  }
  RCLCPP_WARN(
    logger(), "%s 的参数 %s='%s' 不是布尔值，回落为 %s", where.c_str(), key.c_str(),
    it->second.c_str(), fallback ? "true" : "false");
  return fallback;
}

int DmSystemHardware::get_int(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  int fallback, const std::string & where)
{
  return static_cast<int>(get_double(params, key, static_cast<double>(fallback), where));
}

// ------------------------------------------------------------------- on_init
hardware_interface::CallbackReturn DmSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  parse_hardware_parameters();

  joints_.resize(info_.joints.size());
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    if (!parse_joint_config(i, info_.joints[i])) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (info_.joints[i].name == "finger_left") {
      finger_left_index_ = i;
    } else if (info_.joints[i].name == "finger_right") {
      finger_right_index_ = i;
    }
  }

  // 启动前把"会怎么动"打出来：真机联调时这是第一份安全检查清单
  RCLCPP_INFO(
    logger(),
    "初始化完成：device=%s baud=%d mode=%s enable_motors=%s watchdog=%.2fs "
    "velocity_scale=%.2f request_feedback=%s divider=%d 关节数=%zu",
    device_.c_str(), baudrate_, command_mode_.c_str(), enable_motors_ ? "true" : "false",
    watchdog_timeout_, velocity_scale_, request_feedback_ ? "true" : "false",
    feedback_divider_, joints_.size());

  if (command_mode_ == "mit") {
    for (const auto & joint : joints_) {
      if (joint.has_command && (joint.kp <= 0.0 || joint.kd <= 0.0)) {
        RCLCPP_ERROR(
          logger(),
          "MIT 模式下 %s 的 kp/kd 必须为正（当前 kp=%f kd=%f）",
          joint.name.c_str(), joint.kp, joint.kd);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

void DmSystemHardware::parse_hardware_parameters()
{
  const auto & p = info_.hardware_parameters;
  const std::string where = "hardware";

  if (const auto it = p.find("device"); it != p.end()) {
    device_ = it->second;
  }
  baudrate_ = get_int(p, "baudrate", baudrate_, where);
  if (const auto it = p.find("command_mode"); it != p.end()) {
    command_mode_ = it->second;
  }
  // 三种控制模式：
  //   pos_vel    —— 电机内部位置环（默认，最安全；pos 命令来自 JTC）
  //   mit        —— MIT 位置阻抗，kp/kd 由 URDF 参数给定
  //   mit_torque —— 纯力矩：kp=kd=0，tau 来自控制器的 effort 命令（重力补偿用）
  if (command_mode_ != "pos_vel" && command_mode_ != "mit" && command_mode_ != "mit_torque") {
    RCLCPP_WARN(
      logger(),
      "未知 command_mode='%s'，只用 pos_vel / mit / mit_torque；回落 pos_vel",
      command_mode_.c_str());
    command_mode_ = "pos_vel";
  }
  // ⚠ 布尔参数绝不能走数字解析：曾经把 "false" 当非法数字回落成 true，
  //   等于把"只读模式"的安全闸关掉了。这类回落的默认值必须往"更安全"的一侧取。
  enable_motors_ = get_bool(p, "enable_motors", enable_motors_, where);
  watchdog_timeout_ = get_double(p, "watchdog_timeout", watchdog_timeout_, where);
  can_timeout_ms_ = get_int(p, "can_timeout_ms", can_timeout_ms_, where);
  max_write_failures_ = std::max(1, get_int(p, "max_write_failures", max_write_failures_, where));
  feedback_wait_ms_ = std::max(1, get_int(p, "feedback_wait_ms", feedback_wait_ms_, where));
  effort_sign_ = get_double(p, "effort_sign", effort_sign_, where);
  if (effort_sign_ != 1.0 && effort_sign_ != -1.0) {
    RCLCPP_WARN(logger(), "effort_sign=%.2f 不是 ±1，按符号处理", effort_sign_);
    effort_sign_ = effort_sign_ < 0.0 ? -1.0 : 1.0;
  }
  command_sign_ = get_double(p, "command_sign", command_sign_, where);
  if (command_sign_ != 1.0 && command_sign_ != -1.0) {
    RCLCPP_WARN(logger(), "command_sign=%.2f 不是 ±1，按符号处理", command_sign_);
    command_sign_ = command_sign_ < 0.0 ? -1.0 : 1.0;
  }
  velocity_scale_ = get_double(p, "velocity_scale", velocity_scale_, where);
  if (velocity_scale_ <= 0.0 || velocity_scale_ > 1.0) {
    RCLCPP_WARN(
      logger(), "velocity_scale=%.3f 超出 (0,1]，夹到合法范围", velocity_scale_);
    velocity_scale_ = std::clamp(velocity_scale_, 0.01, 1.0);
  }
  request_feedback_ = get_bool(p, "request_feedback", request_feedback_, where);
  feedback_divider_ = std::max(1, get_int(p, "feedback_divider", feedback_divider_, where));
  mode_switch_timeout_ms_ =
    std::max(1, get_int(p, "mode_switch_timeout_ms", mode_switch_timeout_ms_, where));

  // ---- CSV 记录（默认关闭）----
  // 开启方式：<param name="log_csv">/path/to/run.csv</param>（相对路径按进程 cwd 解析）。
  // 为什么放在插件里而不是只靠 /joint_states：插件能在"合理性守卫丢帧之前"拿到电机原始
  // 读数，2026-11-20 的抽风事故就是因为只有过滤后的数据、真峰值丢了。
  if (const auto it = p.find("log_csv"); it != p.end()) {
    log_csv_ = it->second;
  }
  log_rate_hz_ = get_double(p, "log_rate_hz", log_rate_hz_, where);
  if (log_rate_hz_ < 1.0) {
    RCLCPP_WARN(logger(), "log_rate_hz=%.1f 太小，按 1 Hz 处理", log_rate_hz_);
    log_rate_hz_ = 1.0;
  }
  log_period_s_ = 1.0 / log_rate_hz_;
}

bool DmSystemHardware::parse_joint_config(
  std::size_t index, const hardware_interface::ComponentInfo & joint)
{
  auto & cfg = joints_[index];
  cfg.name = joint.name;
  const std::string where = "关节 " + joint.name;
  const auto & p = joint.parameters;

  cfg.has_command = !joint.command_interfaces.empty();

  // 没有命令接口的关节（finger_right）只是"状态观察者"，不需要电机参数
  if (!cfg.has_command && p.find("motor_id") == p.end()) {
    RCLCPP_INFO(
      logger(),
      "%s 无命令接口且未配置 motor_id，按纯状态关节处理", cfg.name.c_str());
    return true;
  }

  if (p.find("motor_id") == p.end()) {
    RCLCPP_ERROR(
      logger(),
      "%s 缺少 <param name=\"motor_id\">：请在 ros2_control.xacro 里补上", where.c_str());
    return false;
  }
  cfg.motor_id = get_int(p, "motor_id", 0, where);
  cfg.feedback_id = get_int(p, "feedback_id", cfg.motor_id + 0x10, where);
  cfg.model = p.count("model") ? p.at("model") : std::string("4310");
  cfg.direction = get_double(p, "direction", 1.0, where);
  cfg.scale = get_double(p, "scale", 1.0, where);
  if (cfg.scale == 0.0) {
    RCLCPP_ERROR(logger(), "%s 的 scale 不能为 0", where.c_str());
    return false;
  }
  cfg.zero_offset = get_double(p, "zero_offset", 0.0, where);
  cfg.kp = get_double(p, "kp", 0.0, where);
  cfg.kd = get_double(p, "kd", 0.0, where);
  cfg.vel_limit = get_double(p, "vel_limit", cfg.vel_limit, where);
  cfg.torque_limit = get_double(p, "torque_limit", cfg.torque_limit, where);
  if (p.find("min_pos") != p.end() && p.find("max_pos") != p.end()) {
    cfg.min_pos = get_double(p, "min_pos", 0.0, where);
    cfg.max_pos = get_double(p, "max_pos", 0.0, where);
    // 静止零位漂移许用值（rad/m）：只用于 on_activate 的超限检查
    cfg.limit_tolerance = get_double(p, "limit_tolerance", 0.05, where);
    // 反馈跳变门限（rad/周期）：0 = 按 vel_limit 自动推算（vel_limit·0.08+0.05）。
    // ⚠ 放宽 vel_limit 会顺带放松这道守卫，所以要"跑得快但守得紧"就显式钉死它。
    cfg.feedback_jump_limit = get_double(p, "feedback_jump_limit", 0.0, where);
    cfg.has_position_limit = cfg.max_pos > cfg.min_pos;
  }
  if (cfg.direction != 1.0 && cfg.direction != -1.0) {
    RCLCPP_WARN(
      logger(),
      "%s 的 direction=%.3f 不是 ±1，按符号处理", where.c_str(), cfg.direction);
    cfg.direction = cfg.direction < 0.0 ? -1.0 : 1.0;
  }
  return true;
}

// -------------------------------------------------------------- 接口导出
std::vector<hardware_interface::StateInterface> DmSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & joint = info_.joints[i];
    for (const auto & iface : joint.state_interfaces) {
      if (iface.name == hardware_interface::HW_IF_POSITION) {
        out.emplace_back(joint.name, iface.name, &joints_[i].state_pos);
      } else if (iface.name == hardware_interface::HW_IF_VELOCITY) {
        out.emplace_back(joint.name, iface.name, &joints_[i].state_vel);
      } else if (iface.name == hardware_interface::HW_IF_EFFORT) {
        out.emplace_back(joint.name, iface.name, &joints_[i].state_effort);
      }
    }
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> DmSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & joint = info_.joints[i];
    for (const auto & iface : joint.command_interfaces) {
      if (iface.name == hardware_interface::HW_IF_POSITION) {
        out.emplace_back(joint.name, iface.name, &joints_[i].cmd);
      } else if (iface.name == hardware_interface::HW_IF_EFFORT) {
        // mit_torque 模式用：控制器写力矩，插件原样转成电机 MIT 帧（kp=kd=0）
        out.emplace_back(joint.name, iface.name, &joints_[i].cmd_effort);
      } else {
        RCLCPP_WARN(
          logger(), "本插件不实现 %s 的命令接口 %s", joint.name.c_str(), iface.name.c_str());
      }
    }
  }
  return out;
}

// -------------------------------------------------------------- on_configure
hardware_interface::CallbackReturn DmSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!open_bus()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 收第一帧真实反馈：既验证接线，也拿到"使能前的位置"。
  // 拿不到就不能继续 —— 没有初始位置的使能等于开环，非常危险。
  if (!wait_for_first_feedback()) {
    close_bus();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 切换控制模式：电机的 CTRL_MODE 寄存器可能与我们要用的模式不一致
  // （本机实测出厂/上次运行为 1=MIT）。模式不对时，写命令会被电机忽略或按错语义解释。
  const bool wants_mit = (command_mode_ == "mit" || command_mode_ == "mit_torque");
  const std::uint32_t wanted_mode =
    wants_mit ? static_cast<std::uint32_t>(abi::Mode::MIT)
              : static_cast<std::uint32_t>(abi::Mode::POS_VEL);
  for (auto & joint : joints_) {
    if (joint.handle == nullptr) {
      continue;
    }
    if (motor_handle_ensure_mode(joint.handle, wanted_mode, static_cast<std::uint32_t>(
        mode_switch_timeout_ms_)) != 0) {
      RCLCPP_WARN(
        logger(), "%s 切换控制模式到 %u 失败：%s（继续尝试，但命令可能不生效）",
        joint.name.c_str(), wanted_mode, abi_error_text().c_str());
    }
  }

  // 模式切换必须读回验证（见 verify_control_mode 注释里的事故记录）
  if (!verify_control_mode(wanted_mode)) {
    close_bus();
    return hardware_interface::CallbackReturn::ERROR;
  }

  open_log_file();   // log_csv 为空时什么也不做

  for (auto & joint : joints_) {
    joint.cmd = joint.state_pos;  // 命令初值 = 实测位置（防跳变）
    joint.command_sent = joint.state_pos;
    if (joint.has_command) {
      RCLCPP_INFO(
        logger(),
        "  %-11s 初值 %+8.4f rad（电机 id=%d model=%s）", joint.name.c_str(),
        joint.state_pos, joint.motor_id, joint.model.c_str());
    }
  }

  RCLCPP_INFO(
    logger(), "硬件已配置（电机未使能），等待 activate");
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool DmSystemHardware::open_bus()
{
  if (bus_open_) {
    return true;
  }

  controller_ = motor_controller_new_dm_serial(device_.c_str(), static_cast<std::uint32_t>(baudrate_));
  if (controller_ == nullptr) {
    RCLCPP_ERROR(
      logger(),
      "打不开 %s（baud=%d）：%s\n"
      "  排查：① ls -l /dev/ttyACM* 是否还在 ② 当前用户是否在 dialout 组 "
      "③ 串口是否被别的程序（含另一个 ros2_control_node）占用",
      device_.c_str(), baudrate_, abi_error_text().c_str());
    return false;
  }
  bus_open_ = true;

  for (auto & joint : joints_) {
    if (joint.motor_id == 0) {
      continue;  // 纯状态关节，没有自己的电机
    }
    joint.handle = motor_controller_add_damiao_motor(
      controller_, static_cast<std::uint16_t>(joint.motor_id),
      static_cast<std::uint16_t>(joint.feedback_id), joint.model.c_str());
    if (joint.handle == nullptr) {
      RCLCPP_ERROR(
        logger(), "注册电机 %s(id=%d) 失败：%s",
        joint.name.c_str(), joint.motor_id, abi_error_text().c_str());
      close_bus();
      return false;
    }
    if (can_timeout_ms_ > 0) {
      motor_handle_set_can_timeout_ms(joint.handle, static_cast<std::uint32_t>(can_timeout_ms_));
    }
  }
  return true;
}

void DmSystemHardware::close_bus()
{
  for (auto & joint : joints_) {
    if (joint.handle != nullptr) {
      motor_handle_free(joint.handle);
      joint.handle = nullptr;
    }
  }
  if (controller_ != nullptr) {
    motor_controller_free(controller_);
    controller_ = nullptr;
  }
  bus_open_ = false;
  enabled_ = false;
}

bool DmSystemHardware::verify_control_mode(std::uint32_t expected_mode)
{
  // 为什么要读回确认：
  //   2026-11-20 真机事故 —— ensure_mode() 返回成功但电机实际仍在 POS_VEL 模式，
  //   插件按 MIT 帧发送命令，电机把前几个字节当成位置 float，立刻猛冲（"抽风"）。
  //   同时反馈帧也按错误模式解码，/joint_states 出现 -12 rad 这种多圈垃圾值。
  //   结论：模式切换必须**读回验证**，对不上就拒绝使能，绝不能盲目发命令。
  constexpr std::uint8_t kRegCtrlMode = 10;
  bool all_ok = true;
  for (auto & joint : joints_) {
    if (joint.handle == nullptr) {
      continue;
    }
    std::uint32_t mode = 0;
    const std::int32_t rc =
      motor_handle_get_register_u32(joint.handle, kRegCtrlMode, 500, &mode);
    if (rc != 0) {
      RCLCPP_ERROR(
        logger(), "%s 读回 CTRL_MODE 失败：%s —— 无法确认控制模式，拒绝使能",
        joint.name.c_str(), abi_error_text().c_str());
      all_ok = false;
      continue;
    }
    if (mode != expected_mode) {
      RCLCPP_ERROR(
        logger(),
        "%s 的 CTRL_MODE 读回是 %u，但我们期望 %u —— 模式切换没生效！"
        "此时发命令会让电机按错误语义解释，绝不能继续。"
        "排查：断电重启电机后重试；或先用 pos_vel 模式把手臂摆到安全位形",
        joint.name.c_str(), mode, expected_mode);
      all_ok = false;
    } else {
      RCLCPP_INFO(logger(), "%s CTRL_MODE=%u ✓", joint.name.c_str(), mode);
    }
  }
  return all_ok;
}

bool DmSystemHardware::wait_for_first_feedback()
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(feedback_wait_ms_);
  while (std::chrono::steady_clock::now() < deadline) {
    poll_state();
    bool all_received = true;
    for (const auto & joint : joints_) {
      if (joint.motor_id != 0 && !joint.ever_received) {
        all_received = false;
        break;
      }
    }
    if (all_received) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  for (const auto & joint : joints_) {
    if (joint.motor_id != 0 && !joint.ever_received) {
      RCLCPP_ERROR(
        logger(),
        "%d ms 内没收到 %s（motor_id=%d feedback_id=%d）的反馈帧。\n"
        "  排查：① 电机是否上电（24 V）② CAN 线/终端电阻 ③ feedback_id 是否配错 "
        "④ 该总线是否还有别的程序在收发",
        feedback_wait_ms_, joint.name.c_str(), joint.motor_id, joint.feedback_id);
    }
  }
  return false;
}

// --------------------------------------------------------------- on_activate
hardware_interface::CallbackReturn DmSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 再收一次最新反馈，确保"命令初值 = 当前真实位置"（configure 到 activate 之间手可能推过）
  poll_state();
  for (auto & joint : joints_) {
    joint.cmd = joint.state_pos;
    joint.command_sent = joint.state_pos;
  }
  // 安全检查：实测位置必须落在软限位内。
  // 为什么不能靠 write() 里的夹紧兜底 —— 那样会在使能瞬间把目标强行拉到限位，
  // 相当于电机自己"弹"回限位，是真机上最危险的意外动作之一。
  bool out_of_range = false;
  for (const auto & joint : joints_) {
    if (!joint.has_position_limit || joint.motor_id == 0) {
      continue;
    }
    // 静止漂移容差：静止顶在机械停止位上时读数可能略微越界（零位漂移，实测约 0.026 rad）。
    // 只在这里放宽；write() 的夹紧仍按真实限位，所以命令下不去。
    const double tol = joint.limit_tolerance;
    if (joint.state_pos < joint.min_pos - tol || joint.state_pos > joint.max_pos + tol) {
      RCLCPP_ERROR(
        logger(),
        "%s 当前 %.4f rad 不在软限位 [%.3f, %.3f] 内（含静止漂移容差 %.3f）—— 拒绝使能。"
        "请先用**手**把手臂移回限位内再重新激活（或用 direction/zero_offset 修正映射，"
        "或确认后放宽 min_pos/max_pos）。",
        joint.name.c_str(), joint.state_pos, joint.min_pos, joint.max_pos, tol);
      out_of_range = true;
    } else if (joint.state_pos < joint.min_pos || joint.state_pos > joint.max_pos) {
      RCLCPP_WARN(
        logger(),
        "%s 当前 %.4f rad 略微越出软限位 [%.3f, %.3f]（在静止漂移容差 %.3f 内，允许激活；"
        "命令会把它拉回限位内）",
        joint.name.c_str(), joint.state_pos, joint.min_pos, joint.max_pos, tol);
    }
  }
  if (out_of_range) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  consecutive_write_failures_ = 0;
  consecutive_empty_reads_ = 0;
  watchdog_warned_ = false;
  has_last_command_ = false;

  if (!enable_motors_) {
    RCLCPP_WARN(
      logger(),
      "enable_motors=false（只读模式）：控制器会正常运行、/joint_states 有真实反馈，"
      "但**不会**给电机发使能或运动命令。确认接线无误后把该参数改成 true 再上真动作。");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  const std::int32_t rc = motor_controller_enable_all(controller_);
  if (rc != 0) {
    RCLCPP_ERROR(
      logger(), "使能电机失败：%s", abi_error_text().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  enabled_ = true;
  RCLCPP_INFO(
    logger(),
    "已使能 %zu 台电机（mode=%s）。第一次真机动作请用 0.1 速度缩放、小幅度目标。",
    joints_.size(), command_mode_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------- on_deactivate
hardware_interface::CallbackReturn DmSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (enabled_ && controller_ != nullptr) {
    const std::int32_t rc = motor_controller_disable_all(controller_);
    if (rc != 0) {
      RCLCPP_ERROR(
        logger(), "失能电机失败：%s", abi_error_text().c_str());
    } else {
      RCLCPP_INFO(logger(), "已失能电机（此时手臂可以被手推动）");
    }
  }
  enabled_ = false;
  close_bus();
  close_log_file();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// --------------------------------------------------------------------- read
void DmSystemHardware::poll_state()
{
  if (controller_ == nullptr) {
    return;
  }
  // ① 显式发请求帧：达妙电机是应答式的，不发请求就永远读不到新数据
  //    （实测：裸读串口 1.5 s 收到 0 字节；发请求后 7 台电机都有反馈）
  if (request_feedback_ && (feedback_tick_++ % feedback_divider_ == 0)) {
    for (auto & joint : joints_) {
      if (joint.handle != nullptr) {
        motor_handle_request_feedback(joint.handle);
      }
    }
  }
  // ② 处理接收队列（不发帧）
  motor_controller_poll_feedback_once(controller_);

  for (auto & joint : joints_) {
    if (joint.handle == nullptr) {
      continue;
    }
    DmMotorState state{};
    if (motor_handle_get_state(joint.handle, &state) != 0 || state.has_value == 0) {
      joint.raw_valid = false;
      continue;
    }
    // 先原样留档（电机侧、未换算、未过滤）：CSV 记录用，排查"抽风"就靠它
    joint.raw_pos_motor = static_cast<double>(state.pos);
    joint.raw_vel_motor = static_cast<double>(state.vel);
    joint.raw_torq_motor = static_cast<double>(state.torq);
    joint.raw_valid = true;
    // 电机量 → 关节量：joint = direction * scale * motor + zero_offset
    // scale 只在夹爪上用得上（电机转 rad，关节走 m）：0.01 m/rad 对应 5 rad ≈ 50 mm 行程
    const double new_pos =
      joint.direction * joint.scale * static_cast<double>(state.pos) + joint.zero_offset;
    const double new_vel = joint.direction * joint.scale * static_cast<double>(state.vel);

    // 合理性守卫（血的教训）：真机上曾出现 /joint_states 跳到 −12 rad 这种值，
    // 控制器据此算出巨大误差 → 力矩饱和 → 手臂抽风。这里在**源头**拦住：
    //   · 单周期跳变超出 (vel_limit·dt·4 + 0.05) → 判为坏帧，保留上次值
    //   · 超出硬限位一大截 → 同样丢弃
    if (joint.ever_received) {
      // 门限：显式配置优先，否则按 vel_limit 推算（vel_limit·0.08 + 0.05）
      const double max_jump = joint.feedback_jump_limit > 0.0
        ? joint.feedback_jump_limit
        : joint.vel_limit * 0.02 * 4.0 + 0.05;
      const bool bad_jump = std::abs(new_pos - joint.state_pos) > max_jump;
      const bool bad_range = joint.has_position_limit &&
        (new_pos < joint.min_pos - 0.3 || new_pos > joint.max_pos + 0.3);
      if (bad_jump || bad_range) {
        joint.reject_count++;
        if (joint.reject_count % 200 == 1) {
          RCLCPP_WARN_THROTTLE(
            logger(), *clock_, 1000,
            "%s 的反馈值不合理（读到 %.3f，上次 %.3f，%s）—— 已丢弃该帧",
            joint.name.c_str(), new_pos, joint.state_pos,
            bad_jump ? "跳变过大" : "超出限位");
        }
        continue;      // 保留上一次的合理值
      }
    }
    joint.state_pos = new_pos;
    joint.state_vel = new_vel;
    joint.reject_count = 0;
    joint.state_effort = effort_sign_ * joint.direction * static_cast<double>(state.torq);
    // ⚠ 必须置位：configure 阶段的 wait_for_first_feedback 与 read() 的"读不到反馈"告警都靠它。
    //   2026-11-21 实测：这里漏了这句，导致数据明明收到了却一直认为"没收到首帧"，
    //   configure 直接失败（真机上表现为"2000 ms 内没收到 jointN 的反馈帧"）。
    joint.ever_received = true;
  }

  // finger_right 没有自己的电机（URDF 里是 mimic），但 /joint_states 里必须有它的值，
  // 否则 robot_state_publisher 算出的夹爪 TF 会缺一半。这里按 mimic 关系镜像左指。
  if (finger_right_index_ != kInvalidIndex && finger_left_index_ != kInvalidIndex) {
    const auto & left = joints_[finger_left_index_];
    auto & right = joints_[finger_right_index_];
    right.state_pos = -left.state_pos;
    right.state_vel = -left.state_vel;
    right.state_effort = 0.0;
    right.ever_received = left.ever_received;
  }
}

hardware_interface::return_type DmSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!bus_open_) {
    return hardware_interface::return_type::ERROR;
  }
  poll_state();

  // CSV 记录（log_csv 非空才写）
  if (log_file_.is_open()) {
    const double dt = period.seconds() > 0.0 ? period.seconds() : log_period_s_;
    log_accum_s_ += dt;
    if (log_accum_s_ >= log_period_s_ - 1e-9) {
      log_accum_s_ = 0.0;
      const double now = clock_->now().seconds();
      if (log_t0_ < 0.0) {
        log_t0_ = now;
      }
      append_log_row(now - log_t0_);
      if (log_rows_ % 200 == 0) {   // 约每 1 s 落盘一次，避免在 RT 周期里频繁 I/O
        flush_log_buffer();
      }
    }
  }

  bool any = false;
  for (const auto & joint : joints_) {
    any = any || joint.ever_received;
  }
  if (!any) {
    ++consecutive_empty_reads_;
    RCLCPP_WARN_THROTTLE(
      logger(), *clock_, 1000,
      "已连续 %d 个周期读不到任何反馈（检查 request_feedback 参数、电机供电与 CAN 线）",
      consecutive_empty_reads_);
  } else {
    consecutive_empty_reads_ = 0;
    last_feedback_time_ = clock_->now();
  }
  return hardware_interface::return_type::OK;
}

// -------------------------------------------------------------------- write
void DmSystemHardware::command_motor(JointConfig & joint, double target, double period)
{
  // 关节目标 → 电机目标（上面映射的逆运算）
  const double motor_target = (target - joint.zero_offset) / (joint.direction * joint.scale);
  // vel_limit 是关节单位，POS_VEL 模式的 vlim 要换成电机单位（rad/s）
  const double motor_vel_limit = std::abs(joint.scale) * joint.vel_limit * velocity_scale_;

  std::int32_t rc = 0;
  if (command_mode_ == "mit") {
    // MIT：位置 + 速度前馈 0 + kp/kd 阻抗 + 力矩前馈 0
    rc = motor_handle_send_mit(
      joint.handle, static_cast<float>(motor_target), 0.0F, static_cast<float>(joint.kp),
      static_cast<float>(joint.kd), 0.0F);
  } else {
    // POS_VEL：电机内部做位置环，我们给速度和位置上限
    rc = motor_handle_send_pos_vel(
      joint.handle, static_cast<float>(motor_target), static_cast<float>(motor_vel_limit));
  }

  if (rc != 0) {
    ++consecutive_write_failures_;
    RCLCPP_WARN_THROTTLE(
      logger(), *clock_, 500,
      "向 %s 写命令失败（连续 %d 次）：%s", joint.name.c_str(), consecutive_write_failures_,
      abi_error_text().c_str());
  } else {
    consecutive_write_failures_ = 0;
  }
  (void)period;
}

hardware_interface::return_type DmSystemHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!bus_open_) {
    return hardware_interface::return_type::ERROR;
  }
  if (!enabled_) {
    return hardware_interface::return_type::OK;  // 只读模式：不写任何命令
  }

  const double dt = std::clamp(period.seconds(), 1e-4, 0.1);

  for (auto & joint : joints_) {
    if (!joint.has_command || joint.handle == nullptr) {
      continue;
    }

    // ---- mit_torque：直接把控制器算好的 τ 发下去（kp=kd=0，纯力矩）----
    if (command_mode_ == "mit_torque") {
      double tau = joint.cmd_effort;
      if (joint.torque_limit > 0.0) {
        tau = std::clamp(tau, -joint.torque_limit, joint.torque_limit);
      }
      // 电机侧 MIT 的 pos/vel 在 kp=kd=0 时不生效，但给当前值更安全
      const std::int32_t rc = motor_handle_send_mit(
        joint.handle, static_cast<float>(joint.direction * (joint.state_pos - joint.zero_offset) /
                                         joint.scale),
        0.0F, 0.0F, 0.0F,
        static_cast<float>(command_sign_ * joint.direction * tau));
      if (rc != 0) {
        ++consecutive_write_failures_;
        RCLCPP_WARN_THROTTLE(logger(), *clock_, 500,
                            "向 %s 写力矩失败（连续 %d 次）：%s", joint.name.c_str(),
                            consecutive_write_failures_, abi_error_text().c_str());
      } else {
        consecutive_write_failures_ = 0;
      }
      continue;
    }

    double target = joint.cmd;

    // 安全 1：软限位夹紧
    if (joint.has_position_limit) {
      target = std::clamp(target, joint.min_pos, joint.max_pos);
    }

    // 安全 2：限速 —— 每周期最多相对"上次真正下发的目标"移动 vel_limit × dt。
    //   基准必须是 command_sent（我们实际写进电机的值），不能是 joint.cmd
    //   （控制器本周期刚写进来的期望值），否则限速形同虚设。
    const double max_step = joint.vel_limit * velocity_scale_ * dt;
    if (std::isfinite(joint.command_sent)) {
      target = std::clamp(target, joint.command_sent - max_step, joint.command_sent + max_step);
    }

    // 安全 3：看门狗 —— 超过 watchdog_timeout 没有新的写周期，就原地保持当前实测位置
    const auto now = std::chrono::steady_clock::now();
    if (watchdog_timeout_ > 0.0) {
      const double since_last =
        std::chrono::duration<double>(now - last_command_time_).count();
      if (has_last_command_ && since_last > watchdog_timeout_) {
        if (!watchdog_warned_) {
          RCLCPP_WARN(
            logger(),
            "看门狗触发（%.2fs 无新命令），%s 保持当前位置", since_last, joint.name.c_str());
          watchdog_warned_ = true;
        }
        target = joint.state_pos;
      }
    }

    command_motor(joint, target, dt);
    joint.command_sent = target;  // 记录实际下发值，作为下一周期限速的基准
  }

  (void)time;
  last_command_time_ = std::chrono::steady_clock::now();
  has_last_command_ = true;

  if (consecutive_write_failures_ >= max_write_failures_) {
    RCLCPP_ERROR(
      logger(),
      "连续 %d 次写命令失败，主动停用硬件以避免失控：%s", consecutive_write_failures_,
      abi_error_text().c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------- CSV 记录
bool DmSystemHardware::open_log_file()
{
  close_log_file();
  if (log_csv_.empty()) {
    return false;
  }
  log_file_.open(log_csv_, std::ios::out | std::ios::trunc);
  if (!log_file_.is_open()) {
    RCLCPP_ERROR(
      logger(), "打不开 CSV 记录文件 '%s'（检查目录是否存在/可写）—— 本次不记录数据",
      log_csv_.c_str());
    return false;
  }
  log_rows_ = 0;
  log_t0_ = -1.0;
  log_accum_s_ = 0.0;

  // 文件头：控制模式 + 关键标定，方便离线分析时不用再去翻 URDF
  log_file_ << "# B601 DmSystemHardware log\n";
  log_file_ << "# device=" << device_ << " command_mode=" << command_mode_
            << " enable_motors=" << (enable_motors_ ? "true" : "false")
            << " velocity_scale=" << velocity_scale_
            << " effort_sign=" << effort_sign_ << " command_sign=" << command_sign_
            << " log_rate_hz=" << log_rate_hz_ << "\n";
  log_file_ << "# 列说明：t=秒(自开始)；每关节 7 列 = "
               "pos_motor(rad) vel_motor(rad/s) torq_motor(N·m) pos_joint vel_joint "
               "effort_joint cmd"
               "（pos_vel/mit 模式下 cmd 是目标位置；mit_torque 模式下 cmd 是力矩命令）\n";
  log_file_ << "# kp/kd（仅 mit 模式生效）：";
  for (const auto & joint : joints_) {
    log_file_ << " " << joint.name << "=" << joint.kp << "/" << joint.kd;
  }
  log_file_ << "\n";

  log_file_ << "t";
  for (const auto & joint : joints_) {
    log_file_ << "," << joint.name << "_pos_motor"
              << "," << joint.name << "_vel_motor"
              << "," << joint.name << "_torq_motor"
              << "," << joint.name << "_pos_joint"
              << "," << joint.name << "_vel_joint"
              << "," << joint.name << "_effort_joint"
              << "," << joint.name << "_cmd";
  }
  log_file_ << "\n";
  log_file_.flush();
  RCLCPP_INFO(
    logger(), "CSV 记录已开启：%s（%.0f Hz，%zu 关节）",
    log_csv_.c_str(), log_rate_hz_, joints_.size());
  return true;
}

void DmSystemHardware::append_log_row(double t_sec)
{
  if (!log_file_.is_open()) {
    return;
  }
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.4f", t_sec);
  log_buffer_ += buf;
  for (const auto & joint : joints_) {
    // 原始值与过滤后的状态值都写：出问题时能一眼看出"是电机在动"还是"数据被丢了"
    const double cmd_value = (command_mode_ == "mit_torque") ? joint.cmd_effort : joint.cmd;
    std::snprintf(
      buf, sizeof(buf), ",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
      joint.raw_pos_motor, joint.raw_vel_motor, joint.raw_torq_motor,
      joint.state_pos, joint.state_vel, joint.state_effort, cmd_value);
    log_buffer_ += buf;
  }
  log_buffer_ += "\n";
  ++log_rows_;
}

void DmSystemHardware::flush_log_buffer()
{
  if (!log_file_.is_open() || log_buffer_.empty()) {
    return;
  }
  log_file_ << log_buffer_;
  log_file_.flush();
  log_buffer_.clear();
}

void DmSystemHardware::close_log_file()
{
  if (!log_file_.is_open()) {
    return;
  }
  flush_log_buffer();
  log_file_.close();
  RCLCPP_INFO(logger(), "CSV 记录已关闭：%s（共 %lu 行）",
    log_csv_.c_str(), static_cast<unsigned long>(log_rows_));
}

}  // namespace b601_hardware

PLUGINLIB_EXPORT_CLASS(b601_hardware::DmSystemHardware, hardware_interface::SystemInterface)
