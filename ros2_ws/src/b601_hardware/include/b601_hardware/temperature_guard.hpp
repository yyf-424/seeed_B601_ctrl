// temperature_guard.hpp —— 电机温度守护（② 层，纯逻辑，不依赖 ROS）
//
// 为什么需要它（2026-11-22 决策③）：
//   变刚度方案里**不做力矩位置保护**（不夹软限位、不做软墙），堵转时唯一持续增长的量就是温度。
//   温度造成的是**不可逆损伤**（绕组/MOS 烧毁），而塌落是可恢复的机械事故 —— 所以温度必须
//   在**硬件层**兜底，不能交给上层节点（安全节点可能没起、被停或掉 DDS）。
//
// 逻辑约定：
//   · 告警线 warn_c：只报警，不动作；
//   · 触发线 limit_c：**立即失能**（整条总线，避免单关节塌）并**锁存**；
//   · 恢复：必须降到 warn_c - hysteresis_c 以下，且**由人重启硬件组件**才解除锁存
//     （自动恢复等于让过热的电机反复上电，是最坏的做法）。
//
// 为什么是纯头文件：两个硬件后端（DmSystemHardware / MujocoSystemHardware）必须用**同一份**
// 判定逻辑，否则"仿真过、真机翻车"（历史上 effort_sign 两后端相反就是这个教训）。

#pragma once

#include <algorithm>
#include <cmath>

namespace b601_hardware
{

class TemperatureGuard
{
public:
  struct Result
  {
    bool warn{false};        // 本次超过告警线（未触发）
    bool trip{false};        // 本次需要立即失能（边沿：只在第一次触发时为 true）
    double max_c{0.0};       // 参与判定的最高温度（t_mos / t_rotor 取大）
  };

  void configure(double warn_c, double limit_c, double hysteresis_c)
  {
    warn_c_ = std::max(0.0, warn_c);
    limit_c_ = std::max(warn_c_ + 1e-6, limit_c);
    hysteresis_c_ = std::max(0.0, hysteresis_c);
  }

  /// 每周期调用一次。t_max 是本次读到的最高温度（℃）。
  /// enabled=false（如只读模式/未使能）时只跟踪、不触发失能 —— 电机本来就没出力。
  Result update(double t_max, bool enabled)
  {
    Result r;
    r.max_c = t_max;
    if (!std::isfinite(t_max) || t_max <= 0.0) {
      // 读不到温度不算故障（反馈帧里 has_value=0 的情况）；不要拿 0 ℃ 当"很凉快"
      return r;
    }
    max_seen_ = std::max(max_seen_, t_max);
    if (latched_) {
      cooled_ = t_max <= (warn_c_ - hysteresis_c_);
      return r;   // 锁存后不再重复触发；恢复必须人工重启硬件组件
    }
    r.warn = t_max >= warn_c_;
    if (t_max >= limit_c_) {
      latched_ = true;
      r.trip = enabled;
    }
    return r;
  }

  bool latched() const {return latched_;}
  bool cooled() const {return cooled_;}
  double max_seen_c() const {return max_seen_;}
  double warn_c() const {return warn_c_;}
  double limit_c() const {return limit_c_;}
  double hysteresis_c() const {return hysteresis_c_;}

private:
  double warn_c_{70.0};
  double limit_c_{85.0};
  double hysteresis_c_{5.0};
  double max_seen_{0.0};
  bool latched_{false};
  bool cooled_{false};
};

}  // namespace b601_hardware
