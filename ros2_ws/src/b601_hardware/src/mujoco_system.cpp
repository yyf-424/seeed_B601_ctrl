#include "b601_hardware/mujoco_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <mujoco/mujoco.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace b601_hardware
{

namespace
{
rclcpp::Logger logger() { return rclcpp::get_logger("MujocoSystemHardware"); }
}  // namespace

double MujocoSystemHardware::get_double(
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
      logger(), "%s 的参数 %s='%s' 不是数字，回落 %.4f", where.c_str(), key.c_str(),
      it->second.c_str(), fallback);
    return fallback;
  }
}

hardware_interface::CallbackReturn MujocoSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hp = info_.hardware_parameters;
  if (const auto it = hp.find("model_path"); it != hp.end()) {
    model_path_ = it->second;
  }
  if (const auto it = hp.find("command_mode"); it != hp.end()) {
    command_mode_ = it->second;
  }
  if (command_mode_ != "pos_vel" && command_mode_ != "mit" && command_mode_ != "mit_torque" &&
    command_mode_ != "mit_impedance")
  {
    RCLCPP_WARN(
      logger(), "未知 command_mode='%s'，回落 pos_vel", command_mode_.c_str());
    command_mode_ = "pos_vel";
  }
  effort_sign_ = get_double(hp, "effort_sign", effort_sign_, "hardware");
  if (effort_sign_ != 1.0 && effort_sign_ != -1.0) {
    RCLCPP_WARN(logger(), "effort_sign=%.2f 不是 ±1，按符号处理", effort_sign_);
    effort_sign_ = effort_sign_ < 0.0 ? -1.0 : 1.0;
  }
  verbose_ = get_double(hp, "verbose", b601_config::mujoco::verbose, "hardware") != 0.0;
  // 温度守护 + 一阶热模型（阈值与真机同一套参数名）
  temp_guard_.configure(
    get_double(hp, "temp_warn_c", b601_config::dm::warn_c, "hardware"),
    get_double(hp, "temp_limit_c", b601_config::dm::limit_c, "hardware"),
    get_double(hp, "temp_hysteresis_c", b601_config::dm::hysteresis_c, "hardware"));
  thermal_ambient_c_ = get_double(hp, "thermal_ambient_c", thermal_ambient_c_, "hardware");
  thermal_heat_c_per_s_ = get_double(hp, "thermal_heat_c_per_s", thermal_heat_c_per_s_, "hardware");
  thermal_cooling_tau_s_ = std::max(1.0, get_double(hp, "thermal_cooling_tau_s", thermal_cooling_tau_s_, "hardware"));
  temp_now_c_ = thermal_ambient_c_;
  RCLCPP_INFO(
    logger(),
    "温度守护（仿真）：告警 %.0f ℃ / 失能 %.0f ℃；热模型 环境 %.0f ℃、满载 %.2f ℃/s、冷却 %.0f s",
    temp_guard_.warn_c(), temp_guard_.limit_c(), thermal_ambient_c_,
    thermal_heat_c_per_s_, thermal_cooling_tau_s_);
  physics_wait_cycles_ = static_cast<long>(
    std::max(1.0, get_double(hp, "physics_wait_cycles", physics_wait_cycles_, "hardware")));

  if (model_path_.empty()) {
    RCLCPP_ERROR(logger(), "缺少 model_path 参数（指向 MJCF 文件）");
    return hardware_interface::CallbackReturn::ERROR;
  }

  joints_.clear();
  for (const auto & joint : info_.joints) {
    JointMap jm;
    jm.name = joint.name;
    jm.has_command = !joint.command_interfaces.empty();
    const auto & p = joint.parameters;
    auto num = [&p](const char * key, double fallback) {
        const auto it = p.find(key);
        if (it == p.end() || it->second.empty()) {
          return fallback;
        }
        try {
          return std::stod(it->second);
        } catch (const std::exception &) {
          return fallback;
        }
      };
    jm.kp = num("kp", 0.0);
    jm.kd = num("kd", 0.0);
    jm.vel_limit = num("vel_limit", jm.vel_limit);
    jm.target_slew = num("target_slew", jm.target_slew);
    jm.torque_limit = num("torque_limit", 0.0);
    const auto lo = p.find("min_pos");
    const auto hi = p.find("max_pos");
    if (lo != p.end() && hi != p.end() && !lo->second.empty() && !hi->second.empty()) {
      jm.min_pos = std::stod(lo->second);
      jm.max_pos = std::stod(hi->second);
      jm.has_position_limit = true;
    }
    // 初始位形：来自 xacro 传的 <param name="initial_position">（与 URDF 的 initial_value 同源）。
    // 为什么必须这么做：MJCF 的 qpos0 全是 0，而 joint3/joint4 的限位正好是 0 / 1.57 ——
    // 从全 0 起步等于贴着限位，而且全 0 是"竖直伸展"位形，仿真里会直接朝限位倒下去
    // （2026-11-21 实测：安全节点第一帧就判急停）。
    jm.init_position = num("initial_position", 0.0);
    jm.cmd_position = jm.init_position;

    const std::size_t index = joints_.size();
    joints_.push_back(jm);
    if (joint.name == "finger_left") {
      finger_left_index_ = index;
    } else if (joint.name == "finger_right") {
      finger_right_index_ = index;
    }
  }

  RCLCPP_INFO(
    logger(), "初始化完成：model=%s mode=%s 关节数=%zu",
    model_path_.c_str(), command_mode_.c_str(), joints_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!load_model()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 关节名 → qpos/dof 索引映射；执行器按 "jointN_motor" 约定找
  for (auto & jm : joints_) {
    const int jid = mj_name2id(model_, mjOBJ_JOINT, jm.name.c_str());
    if (jid < 0) {
      RCLCPP_ERROR(logger(), "MJCF 里找不到关节 %s", jm.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    jm.qpos_adr = model_->jnt_qposadr[jid];
    jm.dof_adr = model_->jnt_dofadr[jid];
    const std::string act_name = jm.name + "_motor";
    jm.actuator = mj_name2id(model_, mjOBJ_ACTUATOR, act_name.c_str());
    jm.is_finger = (jm.actuator < 0);
    const bool is_prismatic_finger = (jm.name == "finger_left" || jm.name == "finger_right");
    const int actuator = jm.actuator;
    jm.implicit_impedance = command_mode_ == "mit_impedance" && is_prismatic_finger &&
      actuator >= 0 && model_->actuator_trntype[actuator] == mjTRN_JOINT &&
      model_->actuator_trnid[2 * actuator] == jid &&
      model_->actuator_dyntype[actuator] == mjDYN_NONE &&
      model_->actuator_gear[6 * actuator] == 1.0;
    if (jm.implicit_impedance) {
      model_->actuator_gaintype[actuator] = mjGAIN_FIXED;
      model_->actuator_gainprm[mjNGAIN * actuator] = 1.0;
      model_->actuator_biastype[actuator] = mjBIAS_AFFINE;
      mju_zero(model_->actuator_biasprm + mjNBIAS * actuator, mjNBIAS);
      model_->actuator_ctrllimited[actuator] = 0;
    }
    if (actuator >= 0 && jm.torque_limit > 0.0) {
      model_->actuator_ctrllimited[actuator] = 0;
      model_->actuator_forcelimited[actuator] = 1;
      model_->actuator_forcerange[2 * actuator] = -jm.torque_limit;
      model_->actuator_forcerange[2 * actuator + 1] = jm.torque_limit;
    }
    jm.passthrough = (jm.actuator < 0) ||
      (is_prismatic_finger && command_mode_ != "mit_torque" &&
       command_mode_ != "mit_impedance");
    if (jm.is_finger && jm.has_command && command_mode_ == "mit_torque") {
      RCLCPP_WARN(
        logger(), "%s 在 MJCF 里没有执行器：imt_torque 模式下它的 effort 命令会被忽略",
        jm.name.c_str());
    }
    // 仿真初值 = URDF 的 initial_value（不是 MJCF 的全 0），并把命令初值设为同一值（防跳变）
    data_->qpos[jm.qpos_adr] = jm.init_position;
    data_->qvel[jm.dof_adr] = 0.0;
    jm.state_position = jm.init_position;
    jm.cmd_position = jm.init_position;
    jm.cmd_sent = jm.init_position;
  }
  mirror_finger();
  mj_forward(model_, data_);
  steps_total_ = 0;
  RCLCPP_INFO(
    logger(), "已加载 MuJoCo 模型：nq=%lld nv=%lld nu=%lld，dt=%.4f s",
    static_cast<long long>(model_->nq), static_cast<long long>(model_->nv),
    static_cast<long long>(model_->nu), model_->opt.timestep);
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool MujocoSystemHardware::load_model()
{
  if (model_ != nullptr) {
    return true;
  }
  char error[1024] = {0};
  model_ = mj_loadXML(model_path_.c_str(), nullptr, error, sizeof(error));
  if (model_ == nullptr) {
    RCLCPP_ERROR(logger(), "加载 MJCF 失败：%s（%s）", model_path_.c_str(), error);
    return false;
  }
  data_ = mj_makeData(model_);
  if (data_ == nullptr) {
    RCLCPP_ERROR(logger(), "mj_makeData 失败");
    mj_deleteModel(model_);
    model_ = nullptr;
    return false;
  }
  return true;
}

hardware_interface::CallbackReturn MujocoSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (model_ == nullptr || data_ == nullptr) {
    RCLCPP_ERROR(logger(), "尚未加载模型，拒绝激活");
    return hardware_interface::CallbackReturn::ERROR;
  }
  // 命令初值 = 当前仿真位形，避免激活瞬间跳变（与真机插件的语义保持一致）
  for (auto & jm : joints_) {
    jm.cmd_position = data_->qpos[jm.qpos_adr];
    jm.cmd_sent = jm.cmd_position;
    jm.cmd_velocity = 0.0;
    jm.cmd_kp = 0.0;
    jm.cmd_kd = 0.0;
    jm.cmd_low_stiffness = 0.0;
    jm.state_low_stiffness = 0.0;
    jm.cmd_effort = 0.0;
  }
  mju_zero(data_->ctrl, model_->nu);
  sim_started_ = false;
  cycles_since_activate_ = 0;
  for (const auto & jm : joints_) {
    RCLCPP_INFO(
      logger(), "  %-11s qpos_adr=%d dof_adr=%d actuator=%d init=%+.3f kp=%.1f kd=%.1f 限位[%.2f,%.2f]",
      jm.name.c_str(), jm.qpos_adr, jm.dof_adr, jm.actuator, jm.init_position, jm.kp, jm.kd,
      jm.min_pos, jm.max_pos);
  }
  RCLCPP_INFO(
    logger(), "已激活（仿真）：模式 %s，%zu 关节（其中 %d 个走运动学直通）",
    command_mode_.c_str(), joints_.size(),
    static_cast<int>(std::count_if(
      joints_.begin(), joints_.end(), [](const JointMap & j) { return j.passthrough; })));
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (model_ != nullptr && data_ != nullptr) {
    mju_zero(data_->ctrl, model_->nu);
    mju_zero(data_->qfrc_actuator, model_->nv);
    sim_started_ = false;
    cycles_since_activate_ = 0;
  }
  RCLCPP_INFO(logger(), "已停用（仿真）：控制量清零");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (data_ != nullptr) {
    mj_deleteData(data_);
    data_ = nullptr;
  }
  if (model_ != nullptr) {
    mj_deleteModel(model_);
    model_ = nullptr;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  const bool with_effort = true;
  for (auto & jm : joints_) {
    states.emplace_back(jm.name, hardware_interface::HW_IF_POSITION, &jm.state_position);
    states.emplace_back(jm.name, hardware_interface::HW_IF_VELOCITY, &jm.state_velocity);
    if (with_effort) {
      states.emplace_back(jm.name, hardware_interface::HW_IF_EFFORT, &jm.state_effort);
    }
    if (command_mode_ == "mit_impedance" && jm.name == "joint1") {
      states.emplace_back(jm.name, "low_stiffness", &jm.state_low_stiffness);
    }
  }
  return states;
}

std::vector<hardware_interface::CommandInterface> MujocoSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> commands;
  for (auto & jm : joints_) {
    if (!jm.has_command) {
      continue;   // finger_right 是 mimic，无命令接口
    }
    if (command_mode_ == "mit_impedance") {
      commands.emplace_back(jm.name, hardware_interface::HW_IF_POSITION, &jm.cmd_position);
      commands.emplace_back(jm.name, hardware_interface::HW_IF_VELOCITY, &jm.cmd_velocity);
      commands.emplace_back(jm.name, "kp", &jm.cmd_kp);
      commands.emplace_back(jm.name, "kd", &jm.cmd_kd);
      commands.emplace_back(jm.name, hardware_interface::HW_IF_EFFORT, &jm.cmd_effort);
      if (jm.name == "joint1") {
        commands.emplace_back(jm.name, "low_stiffness", &jm.cmd_low_stiffness);
      }
    } else if (command_mode_ == "mit_torque") {
      commands.emplace_back(jm.name, hardware_interface::HW_IF_EFFORT, &jm.cmd_effort);
    } else {
      commands.emplace_back(jm.name, hardware_interface::HW_IF_POSITION, &jm.cmd_position);
    }
  }
  return commands;
}

void MujocoSystemHardware::mirror_finger()
{
  if (finger_left_index_ == static_cast<std::size_t>(-1) ||
      finger_right_index_ == static_cast<std::size_t>(-1))
  {
    return;
  }
  const auto & left = joints_[finger_left_index_];
  const auto & right = joints_[finger_right_index_];
  // 从动指没有执行器和命令接口，直接镜像主动指的物理状态。
  data_->qpos[right.qpos_adr] = -data_->qpos[left.qpos_adr];
  data_->qvel[right.dof_adr] = -data_->qvel[left.dof_adr];
}

void MujocoSystemHardware::apply_impedance_inside_physics()
{
  // 模拟"电机内部位置环"：每个物理子步都用**当前** q/qd 重算一次力矩。
  // 这正是电机侧变刚度的价值 —— 环不在 200 Hz 的 ROS 控制周期里，
  // 所以不受反馈延迟影响（真机上这是能否把 kp 提到 120 量级的关键）。
  for (auto & jm : joints_) {
    if (jm.actuator < 0) {
      continue;
    }
    if (!jm.implicit_impedance &&
      jm.cmd_kp <= 0.0 && jm.cmd_kd <= 0.0 && std::abs(jm.cmd_effort) < 1e-9)
    {
      data_->ctrl[jm.actuator] = 0.0;
      continue;
    }
    double pos_des = jm.cmd_position;
    double velocity_des = std::clamp(
      jm.cmd_velocity, -jm.vel_limit, jm.vel_limit);
    if (jm.has_position_limit) {
      pos_des = std::clamp(pos_des, jm.min_pos, jm.max_pos);
      if ((pos_des <= jm.min_pos && velocity_des < 0.0) ||
        (pos_des >= jm.max_pos && velocity_des > 0.0))
      {
        velocity_des = 0.0;
      }
    }
    if (jm.implicit_impedance) {
      // Put the velocity bias in the actuator so MuJoCo can integrate damping implicitly.
      auto * bias = model_->actuator_biasprm + mjNBIAS * jm.actuator;
      bias[0] = 0.0;
      bias[1] = -jm.cmd_kp;
      bias[2] = -jm.cmd_kd;
      data_->ctrl[jm.actuator] = jm.cmd_kp * pos_des + jm.cmd_kd * velocity_des +
        effort_sign_ * jm.cmd_effort;
      continue;
    }
    const double q = data_->qpos[jm.qpos_adr];
    const double qd = data_->qvel[jm.dof_adr];
    const double tau = jm.cmd_kp * (pos_des - q) + jm.cmd_kd * (velocity_des - qd) +
      effort_sign_ * jm.cmd_effort;
    data_->ctrl[jm.actuator] = tau;
  }
}

double MujocoSystemHardware::applied_effort(const JointMap & joint) const
{
  if (joint.actuator < 0) {
    return 0.0;
  }
  return data_->actuator_force[joint.actuator];
}

void MujocoSystemHardware::step_sim(double dt)
{
  const double model_dt = model_->opt.timestep;
  int steps = static_cast<int>(std::lround(dt / model_dt));
  if (steps < 1) {
    steps = 1;
  }
  steps = std::min(steps, b601_config::mujoco::max_substeps);
  for (int k = 0; k < steps; ++k) {
    if (command_mode_ == "mit_impedance") {
      apply_impedance_inside_physics();
    }
    mj_step(model_, data_);
    mirror_finger();
  }
  steps_total_ += steps;
}

hardware_interface::return_type MujocoSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (model_ == nullptr || data_ == nullptr) {
    return hardware_interface::return_type::ERROR;
  }
  for (auto & jm : joints_) {
    jm.state_position = data_->qpos[jm.qpos_adr];
    jm.state_velocity = data_->qvel[jm.dof_adr];
    jm.state_effort = applied_effort(jm);
    jm.state_low_stiffness = jm.cmd_low_stiffness;
  }
  if (verbose_ && steps_total_ < b601_config::mujoco::startup_log_steps) {
    std::string line;
    char buf[128];
    for (const auto & jm : joints_) {
      std::snprintf(buf, sizeof(buf), " %s: q=%+.3f cmd=%+.3f", jm.name.c_str(),
        jm.state_position, jm.cmd_position);
      line += buf;
    }
    RCLCPP_INFO(logger(), "[read #%ld]%s", steps_total_, line.c_str());
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (model_ == nullptr || data_ == nullptr) {
    return hardware_interface::return_type::ERROR;
  }
  const double dt = std::clamp(period.seconds(), b601_config::mujoco::write_period_min_sec,
    b601_config::mujoco::write_period_max_sec);

  // ---- 温度守护（决策③）----
  {
    // 一阶热模型：按"实际下达的力矩 / 该关节力矩上限"的比例发热
    double load = 0.0;
    for (const auto & jm : joints_) {
      if (jm.actuator < 0 || jm.torque_limit <= 0.0) {
        continue;
      }
      const double frac = std::abs(applied_effort(jm)) / jm.torque_limit;
      load = std::max(load, frac);
    }
    const double cooling = (temp_now_c_ - thermal_ambient_c_) / thermal_cooling_tau_s_;
    temp_now_c_ = std::max(
      thermal_ambient_c_,
      temp_now_c_ + (thermal_heat_c_per_s_ * load - cooling) * dt);
    const auto r = temp_guard_.update(temp_now_c_, true);
    if (r.trip) {
      over_temp_ = true;
      RCLCPP_ERROR(
        logger(),
        "🛑 过温保护（仿真热模型）：%.1f ℃ ≥ 上限 %.0f ℃ → 失能并让硬件置 error。"
        "恢复需人工重启硬件组件",
        temp_now_c_, temp_guard_.limit_c());
      for (auto & jm : joints_) {
        if (jm.actuator >= 0) {
          data_->ctrl[jm.actuator] = 0.0;
        }
      }
      mju_zero(data_->qfrc_actuator, model_->nv);
      return hardware_interface::return_type::ERROR;
    }
    if (r.warn && !temp_warned_) {
      temp_warned_ = true;
      RCLCPP_WARN(
        logger(), "温度 %.1f ℃ 超过告警线 %.0f ℃（失能线 %.0f ℃）",
        temp_now_c_, temp_guard_.warn_c(), temp_guard_.limit_c());
    } else if (temp_warned_ && temp_now_c_ < temp_guard_.warn_c() - temp_guard_.hysteresis_c()) {
      temp_warned_ = false;
    }
  }

  if (over_temp_) {
    return hardware_interface::return_type::ERROR;
  }

  if ((command_mode_ == "mit_torque" || command_mode_ == "mit_impedance") && !sim_started_) {
    bool any_torque = false;
    for (const auto & jm : joints_) {
      if (jm.actuator >= 0 &&
        (std::abs(jm.cmd_effort) > 1e-6 || (command_mode_ == "mit_impedance" && jm.cmd_kp > 1e-6))) {
        any_torque = true;
        break;
      }
    }
    ++cycles_since_activate_;
    if (!any_torque && cycles_since_activate_ < physics_wait_cycles_) {
      return hardware_interface::return_type::OK;   // 控制器尚未接管：抱闸，不积分
    }
    sim_started_ = true;
    RCLCPP_INFO(
      logger(), "仿真开始积分物理（%s，等待了 %ld 个周期）",
      any_torque ? "收到控制器力矩命令" : "超时兜底", cycles_since_activate_);
  }

  // 位置模式必须补重力项（qfrc_bias）：
  //   真机的 pos_vel 模式是**电机内部位置环**在顶着重力保持，而我们这里是自己做 PD。
  //   只做纯 PD 的话手臂会一路下垂到限位 —— 2026-11-21 实测：joint3 掉到 0（正好是限位），
  //   安全节点立刻判越限并急停，轨迹被中断（error: Current goal cancelled during deactivate）。
  //   mit_torque 模式**不加**这一项：那时控制律应该完全由控制器决定（这样才和真机一致）。
  if (command_mode_ != "mit_torque") {
    mj_forward(model_, data_);     // 刷新 qfrc_bias（重力+科氏）到当前位形
  }

  for (auto & jm : joints_) {
    if (jm.passthrough) {
      // 运动学直通：命令即位置（无执行器的手指 / 位置模式下的夹爪）
      if (jm.has_command) {
        double pos = jm.cmd_position;
        if (jm.has_position_limit) {
          pos = std::clamp(pos, jm.min_pos, jm.max_pos);
        }
        data_->qpos[jm.qpos_adr] = pos;
        data_->qvel[jm.dof_adr] = 0.0;
      }
      continue;
    }

    double tau = 0.0;
    if (command_mode_ == "mit_impedance") {
      // 位置环在驱动器里：这里不写 ctrl，交给 apply_impedance_inside_physics()
      // 在每个物理子步按当前 q/qd 算力矩（等价于 1 kHz 的电机内环）
      continue;
    }
    if (command_mode_ == "mit_torque") {
      tau = effort_sign_ * jm.cmd_effort;   // 与真机插件同一符号约定
    } else {
      // 位置模式：内部 PD + 每周期限速（与真机插件的限速语义一致，基准是上次下发值）
      double target = jm.cmd_position;
      if (jm.has_position_limit) {
        target = std::clamp(target, jm.min_pos, jm.max_pos);
      }
      const double max_step = jm.target_slew * dt;
      if (std::isfinite(jm.cmd_sent)) {
        target = std::clamp(target, jm.cmd_sent - max_step, jm.cmd_sent + max_step);
      }
      jm.cmd_sent = target;
      const double err = target - jm.state_position;
      tau = jm.kp * err - jm.kd * jm.state_velocity + data_->qfrc_bias[jm.dof_adr];
    }
    data_->ctrl[jm.actuator] = tau;
  }

  step_sim(dt);

  // 手指在 MJCF 里没有执行器（是自由滑动关节）：手臂一加速就会被惯性甩出去，
  // 而真机的手指是被电机夹住的。这里做"运动学夹紧"：每周期把手指 qpos 夹回限位并清速度。
  // 2026-11-21 实测：不夹的话 finger_left 会掉到 -0.0000（限位 0 的负侧），
  // 安全节点立刻判越限并急停，整条轨迹被打断。
  for (auto & jm : joints_) {
    if (!jm.passthrough) {
      continue;   // 有执行器的关节由物理决定位置，不夹紧
    }
    double pos = data_->qpos[jm.qpos_adr];
    if (jm.has_position_limit) {
      pos = std::clamp(pos, jm.min_pos, jm.max_pos);
    }
    data_->qpos[jm.qpos_adr] = pos;
    data_->qvel[jm.dof_adr] = 0.0;
  }

  if (verbose_ && steps_total_ % b601_config::mujoco::diagnostic_log_steps == 0) {
    RCLCPP_INFO(
      logger(), "[sim] t=%.2fs q2=%.3f q3=%.3f q6=%.3f",
      steps_total_ * model_->opt.timestep, data_->qpos[joints_[1].qpos_adr],
      data_->qpos[joints_[2].qpos_adr], data_->qpos[joints_[5].qpos_adr]);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace b601_hardware

PLUGINLIB_EXPORT_CLASS(b601_hardware::MujocoSystemHardware, hardware_interface::SystemInterface)
