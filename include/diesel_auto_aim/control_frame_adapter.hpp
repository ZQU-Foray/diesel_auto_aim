#ifndef DIESEL_AUTO_AIM_CONTROL_FRAME_ADAPTER_HPP
#define DIESEL_AUTO_AIM_CONTROL_FRAME_ADAPTER_HPP

#include <cstdint>
#include <optional>
#include <vector>

#include "diesel_auto_aim/gimbal_command.hpp"

namespace diesel::platform::link {
class RoboLinkHostLink;
struct GimbalAttitude;
} // namespace diesel::platform::link

namespace diesel_auto_aim {

// 将云台系相对指向角按当前姿态转换为世界系目标角后组帧；姿态无效则拒绝。
// 本接口不做图像与姿态的时间对齐，调用方必须提供同一时刻的姿态。
std::optional<std::vector<std::uint8_t>>
make_gimbal_control_frame(const GimbalCommand& command,
                          const diesel::platform::link::GimbalAttitude& attitude,
                          std::uint32_t control_seq, std::uint16_t frame_seq,
                          std::uint16_t message_seq, std::uint32_t timestamp_us);

// 运行层入口：链路未握手或没有有效状态回读时拒绝排队。
// 成功只表示进入本机发送队列，不表示 MCU 已接收或执行。
bool queue_gimbal_command(diesel::platform::link::RoboLinkHostLink& link,
                          const GimbalCommand& command,
                          const diesel::platform::link::GimbalAttitude& attitude,
                          std::uint64_t now_ms);

} // namespace diesel_auto_aim

#endif
