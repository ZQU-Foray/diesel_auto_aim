#include "diesel_vision_backend/control_frame_adapter.hpp"

#include <cmath>
#include <limits>

#include <Eigen/Geometry>

#include "diesel/platform/link/robolink_codec.hpp"
#include "diesel/platform/link/robolink_host_link.hpp"

namespace diesel_vision_backend {
namespace {

std::optional<diesel::platform::link::ControlSnapshot>
to_control_snapshot(const GimbalCommand& command,
                    const diesel::platform::link::GimbalAttitude& attitude) {
    if (!command.valid || !std::isfinite(command.yaw) || !std::isfinite(command.pitch))
        return std::nullopt;
    // v0.2 §4.8：只有姿态有效且 IMU READY 时才把相对角转换为世界系目标角。
    if (!attitude.attitude_valid || attitude.imu_state != 1)
        return std::nullopt;

    const auto& components = attitude.quaternion_wxyz;
    for (const float component : components) {
        if (!std::isfinite(component))
            return std::nullopt;
    }
    Eigen::Quaterniond q_gimbal_to_world(components[0], components[1], components[2],
                                         components[3]);
    const double norm = q_gimbal_to_world.norm();
    // 单位四元数允许少量 f32 误差；明显非单位的姿态不能用来控制电机。
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3)
        return std::nullopt;
    q_gimbal_to_world.normalize();

    // aim_at() 的 yaw/pitch 是云台系相对方向，先还原单位方向向量。
    const double cos_pitch = std::cos(command.pitch);
    const Eigen::Vector3d direction_in_gimbal(cos_pitch * std::cos(command.yaw),
                                              cos_pitch * std::sin(command.yaw),
                                              -std::sin(command.pitch));
    const Eigen::Vector3d direction_in_world = q_gimbal_to_world * direction_in_gimbal;
    const double horizontal = std::hypot(direction_in_world.x(), direction_in_world.y());
    if (!direction_in_world.allFinite() || horizontal < 1e-6)
        return std::nullopt;
    const double world_yaw = std::atan2(direction_in_world.y(), direction_in_world.x());
    const double world_pitch = -std::atan2(direction_in_world.z(), horizontal);

    if (std::abs(world_yaw) > std::numeric_limits<float>::max() ||
        std::abs(world_pitch) > std::numeric_limits<float>::max())
        return std::nullopt;

    diesel::platform::link::ControlSnapshot snapshot;
    snapshot.source = 2;     // 候选协议 §4.7：PC 来源。
    snapshot.mode = 2;       // 候选协议 §4.6：VISION_TRACK 期望模式。
    snapshot.flags = 0x0002; // 只声明云台目标，不使能底盘或发射。
    snapshot.gimbal_yaw_rad = static_cast<float>(world_yaw);
    snapshot.gimbal_pitch_rad = static_cast<float>(world_pitch);
    return snapshot;
}

} // namespace

std::optional<std::vector<std::uint8_t>>
make_gimbal_control_frame(const GimbalCommand& command,
                          const diesel::platform::link::GimbalAttitude& attitude,
                          std::uint32_t control_seq, std::uint16_t frame_seq,
                          std::uint16_t message_seq, std::uint32_t timestamp_us) {
    auto snapshot = to_control_snapshot(command, attitude);
    if (!snapshot)
        return std::nullopt;
    snapshot->control_seq = control_seq;
    // 其余字段保持零；是否生效由电控裁决，并通过 SYSTEM_STATUS 回读。
    return diesel::platform::link::make_control_snapshot_frame(frame_seq, timestamp_us, message_seq,
                                                               *snapshot);
}

bool queue_gimbal_command(diesel::platform::link::RoboLinkHostLink& link,
                          const GimbalCommand& command,
                          const diesel::platform::link::GimbalAttitude& attitude,
                          std::uint64_t now_ms) {
    const auto snapshot = to_control_snapshot(command, attitude);
    return snapshot && link.queue_control(*snapshot, now_ms);
}

} // namespace diesel_vision_backend
