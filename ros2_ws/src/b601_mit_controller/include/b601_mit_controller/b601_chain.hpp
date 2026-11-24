// 本文件由 scripts/gen_chain_header.py 从 URDF 自动生成，请勿手改。
// 用途：MIT 控制器在实时循环里算重力力矩，需要固定的串链常量。
#pragma once
#include <array>
#include <cstddef>

namespace b601_mit_controller {
namespace chain {

constexpr std::size_t kNumJoints = 6;

struct JointOrigin {
  const char * name;
  double xyz[3];      // 父 link 系下的平移
  double rot[9];      // 父 link 系下的固定旋转（3x3 行主序）
  double axis[3];     // 单位轴（关节系）
};
// 注意：这里存旋转矩阵而不是 rpy —— joint6 的 rpy=(0,π/2,0) 正好落在万向锁上，
// 反解成 rpy 会得到等价但不同的三元组，容易在 C++ 侧的重构里引入误差。

inline const std::array<JointOrigin, kNumJoints> kJoints = {{
    {"joint1", {-8.416e-05, 0, 0.08465}, {1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 1}},
    {"joint2", {0.020084, 0.031625, 0.05555}, {1, 0, 0, 0, -3.6732051e-06, 1, 0, -1, -3.6732051e-06}, {0, 0, -1}},
    {"joint3", {-0.264, 0, 0}, {1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 1}},
    {"joint4", {0.2426, -0.054, -0.001625}, {1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 1}},
    {"joint5", {0.078308, -0.0375, -0.03}, {1, 0, 0, 0, -3.6732051e-06, 1, 0, -1, -3.6732051e-06}, {0, 0, 1}},
    {"joint6", {0.028008, 0, 0.04}, {-3.6732051e-06, 0, 1, 0, 1, 0, -1, 0, -3.6732051e-06}, {0, 0, 1}},
}};

// 参与重力计算/辨识的 link（顺序必须与 identify.py 输出的参数数组一致）
struct GravityLink {
  const char * name;
  int joint_depth;    // 挂在第几个可动关节（0-based）的子 link 之下
  double xyz[3];      // 相对该子 link 的固定平移
  double rot[9];      // 相对该子 link 的固定旋转（3x3 行主序）
};

inline const std::array<GravityLink, 8> kGravityLinks = {{
    {"link2", 1, {0, 0, 0}, {1, 0, 0, 0, 1, 0, 0, 0, 1}},
    {"link3", 2, {0, 0, 0}, {1, 0, 0, 0, 1, 0, 0, 0, 1}},
    {"link4", 3, {0, 0, 0}, {1, 0, 0, 0, 1, 0, 0, 0, 1}},
    {"link5", 4, {0, 0, 0}, {1, 0, 0, 0, 1, 0, 0, 0, 1}},
    {"link6", 5, {0, 0, 0}, {1, 0, 0, 0, 1, 0, 0, 0, 1}},
    {"end_link", 5, {0, 0, 0.15539}, {3.67320509e-06, -9.26535897e-05, 0.999999996, -3.40335638e-10, -0.999999996, -9.26535897e-05, 1, 0, -3.6732051e-06}},
    {"finger_left_link", 5, {0, 0, 0.15539}, {3.67320509e-06, -9.26535897e-05, 0.999999996, -3.40335638e-10, -0.999999996, -9.26535897e-05, 1, 0, -3.6732051e-06}},
    {"finger_right_link", 5, {0, 0, 0.15539}, {3.67320509e-06, -9.26535897e-05, 0.999999996, -3.40335638e-10, -0.999999996, -9.26535897e-05, 1, 0, -3.6732051e-06}},
}};

constexpr std::size_t kNumGravityLinks = 8;

}  // namespace chain
}  // namespace b601_mit_controller
