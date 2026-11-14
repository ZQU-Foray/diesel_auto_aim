#ifndef DIESEL_VISION_BACKEND_AIMER_HPP
#define DIESEL_VISION_BACKEND_AIMER_HPP

#include "diesel_vision_backend/gimbal_command.hpp"
#include <Eigen/Dense>

namespace diesel_vision_backend {

/// @brief 由目标在云台系下的位置解算云台指向角 直瞄
/// @param xyz_in_gimbal 目标位置 云台系 x 前 y 左 z 上 单位米
/// @param bullet_speed 弹速 m/s 接口预留 激光头测算法暂不做弹道
/// @return 云台角指令 yaw 左正 pitch 下正 单位弧度
GimbalCommand aim_at(const Eigen::Vector3d& xyz_in_gimbal, double bullet_speed);

} // namespace diesel_vision_backend

#endif // DIESEL_VISION_BACKEND_AIMER_HPP
