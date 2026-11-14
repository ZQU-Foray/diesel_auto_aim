#ifndef FORAY_AUTO_AIM_TRACKED_AIM_HPP
#define FORAY_AUTO_AIM_TRACKED_AIM_HPP

#include <Eigen/Geometry>

#include "diesel_vision_backend/gimbal_command.hpp"
#include "diesel_vision_backend/target.hpp"

namespace diesel_vision_backend {

/// 离线/仿真用：由整车估计预测一块板在稳定坐标系中的未来位置。
/// armor_id 是 Tracker 的局部板号，不是机器人编号；dt_s 从观测时刻向前预测。
/// v1 只接受 0–1 秒的离线提前量；超过此范围拒绝，不对远期匀速假设作承诺。
Eigen::Vector3d predict_tracked_armor(const TargetState& state, int armor_id, double dt_s);

/// 将预测板位置变成当前云台系直瞄角；不决定开火，不做弹道或时钟同步。
/// q_gimbal_to_world 与 origin_world 必须描述下发时刻的云台姿态/原点。
GimbalCommand aim_at_tracked_armor(const TargetState& state, int armor_id, double dt_s,
                                   const Eigen::Quaterniond& q_gimbal_to_world,
                                   const Eigen::Vector3d& origin_world, double bullet_speed);

/// 选板结果。"关联到哪块板"与"该打哪块板"是两件事，本函数只回答后者。
struct AimSelection {
    int armor_id = -1;            ///< 选中的局部板号；-1 表示没有可打的板
    double facing = 0.0;          ///< 该板法向与视线的夹角余弦，1 = 正对
    bool hysteresis_kept = false; ///< true 表示因为滞回保留了上一块板

    bool valid() const {
        return armor_id >= 0;
    }
};

/// 在"云台到位时刻"重新选板：与 Tracker 的本帧关联结果无关。
/// 判据是板面法向与"板→云台"视线的夹角余弦（facing），并对上一块板做滞回。
/// @param gimbal_origin_world 云台原点（稳定坐标系），与 predict_tracked_armor 同一坐标系
/// @param dt_s 观测时刻到云台到位时刻的提前量；超出 predict_tracked_armor 的范围会返回无效
/// @param previous_id 上一帧选中的板号，-1 表示没有
/// @param facing_gate_rad 可打的最大夹角；超过此角度视为"侧对"，不选
/// @param hysteresis facing 差值小于该值且上一块仍可打时，保留上一块，避免来回切换
AimSelection select_aim_armor(const TargetState& state, const Eigen::Vector3d& gimbal_origin_world,
                              double dt_s, int previous_id = -1,
                              double facing_gate_rad = 60.0 * M_PI / 180.0,
                              double hysteresis = 0.10);

} // namespace diesel_vision_backend

#endif // FORAY_AUTO_AIM_TRACKED_AIM_HPP
