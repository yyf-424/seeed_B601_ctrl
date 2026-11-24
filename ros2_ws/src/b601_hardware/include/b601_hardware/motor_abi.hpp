// motor_abi.hpp —— 厂商 motor_abi 动态库的 C 接口声明
//
// 为什么直接链这个库，而不是自己实现达妙 CAN 协议：
//   - 达妙电机经 USB-CAN 串口桥通信时，帧封装/使能序列/反馈解析都在这个库里，
//     它是官方 Python SDK（reBotArm_control_py）的传输层依赖 motorbridge 的内核；
//   - 该库导出的是纯 C ABI（见 nm -D 符号），从 C++ 调用是官方支持路径；
//   - 自己重写协议要面对寄存器映射、MIT 定点打包、超时重传等一堆细节，
//     且没有任何收益 —— ros2_control 关心的是"命令写进去、状态读出来"，不是帧格式。
//
// 声明来源：motorbridge/abi.py 里 ctypes 的 argtypes/restype，一一对应。
// 库的查找顺序见 CMakeLists.txt（motor_abi → motorbridge 包内 libmotor_abi.so）。
//
// ⚠ 这个头文件只做声明，不 include 任何 ROS 头，方便单独做单元测试/离线桩。

#pragma once

#include <cstdint>

extern "C" {

// 与 motorbridge/abi.py 的 CState 布局必须逐字段一致（c_int32, c_uint8, c_uint32, c_uint8, 5×c_float）
struct DmMotorState {
  std::int32_t has_value;   // 0 = 还没收到过该电机的反馈帧
  std::uint8_t can_id;
  std::uint32_t arbitration_id;
  std::uint8_t status_code;
  float pos;               // rad（多圈，含圈数）
  float vel;               // rad/s
  float torq;              // N·m
  float t_mos;             // ℃
  float t_rotor;           // ℃
};

// ---- 控制器（一条总线上的一个驱动实例）----
void * motor_controller_new_dm_serial(const char * serial_port, std::uint32_t baud);
void * motor_controller_new_socketcan(const char * channel);
void motor_controller_free(void * controller);
std::int32_t motor_controller_poll_feedback_once(void * controller);  // 只处理接收队列，不发帧
std::int32_t motor_controller_enable_all(void * controller);
std::int32_t motor_controller_disable_all(void * controller);
std::int32_t motor_controller_shutdown(void * controller);
std::int32_t motor_controller_close_bus(void * controller);

// ---- 电机句柄（挂到控制器上的单台电机）----
void * motor_controller_add_damiao_motor(void * controller, std::uint16_t motor_id,
                                        std::uint16_t feedback_id, const char * model);
void motor_handle_free(void * motor);
std::int32_t motor_handle_enable(void * motor);
std::int32_t motor_handle_disable(void * motor);
std::int32_t motor_handle_clear_error(void * motor);
std::int32_t motor_handle_set_zero_position(void * motor);
std::int32_t motor_handle_ensure_mode(void * motor, std::uint32_t mode, std::uint32_t timeout_ms);

// 命令写入（mode = 1: MIT；2: POS_VEL）
std::int32_t motor_handle_send_mit(void * motor, float pos, float vel, float kp, float kd, float tau);
std::int32_t motor_handle_send_pos_vel(void * motor, float pos, float vlim);
std::int32_t motor_handle_request_feedback(void * motor);
std::int32_t motor_handle_set_can_timeout_ms(void * motor, std::uint32_t timeout_ms);

// 状态读取
std::int32_t motor_handle_get_state(void * motor, DmMotorState * out);

// 寄存器读写（用于读回 CTRL_MODE 等配置，确认写入真的生效）
std::int32_t motor_handle_write_register_f32(void * motor, std::uint8_t rid, float value);
std::int32_t motor_handle_write_register_u32(void * motor, std::uint8_t rid, std::uint32_t value);
std::int32_t motor_handle_get_register_f32(void * motor, std::uint8_t rid,
                                          std::uint32_t timeout_ms, float * out);
std::int32_t motor_handle_get_register_u32(void * motor, std::uint8_t rid,
                                          std::uint32_t timeout_ms, std::uint32_t * out);

// 错误信息（线程局部字符串，调用后立即拷贝）
const char * motor_last_error_message();
const char * motor_abi_version();

}  // extern "C"

namespace b601_hardware
{
namespace abi
{
// motorbridge 的 Mode 枚举（models.py）
enum class Mode : std::uint32_t {
  MIT = 1,
  POS_VEL = 2,
  VEL = 3,
  FORCE_POS = 4,
};
}  // namespace abi
}  // namespace b601_hardware
