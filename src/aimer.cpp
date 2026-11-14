#include "diesel_vision_backend/aimer.hpp"

#include <cmath>

namespace diesel_vision_backend {
// 这个子弹解算不管 现在还没有
GimbalCommand aim_at(const Eigen::Vector3d& xyz_in_gimbal, double bullet_speed) {
    (void)bullet_speed; // 激光头测算法 弹道解算后续单独做

    const double x = xyz_in_gimbal.x();
    const double y = xyz_in_gimbal.y();
    const double z = xyz_in_gimbal.z();

    if (!xyz_in_gimbal.allFinite())
        return {0.0, 0.0, false};

    const double d = std::sqrt(x * x + y * y);

    // 目标几乎在正上正下方时水平距离退化 方向无意义
    if (!std::isfinite(d) || d < 1e-6)
        return {0.0, 0.0, false};

    GimbalCommand command;
    command.yaw = std::atan2(y, x);
    command.pitch = -std::atan2(z, d); // 约定向下为正 而 z 朝上
    command.valid = true;              // 可信度判定待实现
    return command;
}

} // namespace diesel_vision_backend
