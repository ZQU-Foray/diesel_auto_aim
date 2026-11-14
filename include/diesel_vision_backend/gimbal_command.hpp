#ifndef DIESEL_VISION_BACKEND_GIMBAL_COMMAND_HPP
#define DIESEL_VISION_BACKEND_GIMBAL_COMMAND_HPP

namespace diesel_vision_backend {

/// 云台系相对指向角，不是 RoboLink v0.2 要求的世界系目标角。
/// 约定 yaw 向左为正 pitch 向下为正；云台系 x 前 y 左 z 上。
struct GimbalCommand {
    double yaw;   ///< 偏航角 rad 向左为正
    double pitch; ///< 俯仰角 rad 向下为正
    bool valid;   ///< false 解算不可信 上层不得据此接管云台
};

} // namespace diesel_vision_backend

#endif // DIESEL_VISION_BACKEND_GIMBAL_COMMAND_HPP
