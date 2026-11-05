#ifndef DIESEL_AUTO_AIM_VISION_ADAPTER_HPP
#define DIESEL_AUTO_AIM_VISION_ADAPTER_HPP

#include <array>

#include "diesel_auto_aim/aimer.hpp"
#include "diesel_auto_aim/gimbal_command.hpp"

namespace diesel_auto_aim {

// 前端检测器输出的一帧观测（过渡期 POD）。
// 正式契约是 diesel_interfaces 的 Observation 消息；I6 落地后本结构只保留
// 消息到内部类型的转换职责，检测类型不再由后端定义。
struct Observation {
    bool pose_success{false};
    std::array<float, 3> t_armor2gimbal{0.0f, 0.0f, 0.0f};
};

// 把前端观测的云台系平移转成直瞄指令；pose_success 为 false 时返回无效指令。
GimbalCommand adapter(const Observation& observation, double bullet_speed);

} // namespace diesel_auto_aim

#endif
