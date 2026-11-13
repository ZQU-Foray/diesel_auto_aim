#ifndef FORAY_AUTO_AIM_WORLD_AIM_SOLVER_HPP
#define FORAY_AUTO_AIM_WORLD_AIM_SOLVER_HPP

#include "diesel_auto_aim/target.hpp"

namespace diesel_auto_aim {

// 解算输入 全部在同一个稳定世界系下 时间单位秒 长度单位米
struct WorldAimRequest {
    TargetState target{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0, 0.0, 0.0, 0.0, 0.0};
    int armor_id = -1; // Tracker推测的局部板号 不是模型识别的类别
    double observation_stamp_s = 0.0;
    double prediction_s = 0.0; // 从观测时刻向后预测 不包含未建模的弹丸飞行时间
    Eigen::Vector3d gimbal_origin_world = Eigen::Vector3d::Zero();
    double bullet_speed_m_s = 0.0; // 给弹道队友预留 直瞄不使用
};

// 世界系绝对角 不能当成云台系相对角再旋转一次
struct WorldAimResult {
    double yaw_rad = 0.0;   // 左正
    double pitch_rad = 0.0; // 下正
    bool valid = false;
};

/// 直瞄与队友弹道的离线测试边界 不负责串口和开火
class WorldAimSolver {
  public:
    virtual ~WorldAimSolver() = default;
    /// 输入相同时刻的整车状态 返回世界系绝对角 单位rad 无效必须返回valid=false
    virtual WorldAimResult solve(const WorldAimRequest& request) const = 0;
};

/// 当前只做几何直瞄 不实现弹道补偿 队友另写实现后通过上面的接口测试
class DirectWorldAimSolver final : public WorldAimSolver {
  public:
    WorldAimResult solve(const WorldAimRequest& request) const override;
};

} // namespace diesel_auto_aim

#endif // FORAY_AUTO_AIM_WORLD_AIM_SOLVER_HPP
