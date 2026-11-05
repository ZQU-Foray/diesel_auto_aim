#include "diesel_auto_aim/tracked_aim.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "diesel_auto_aim/aimer.hpp"

namespace diesel_auto_aim {

Eigen::Vector3d predict_tracked_armor(const TargetState& state, int armor_id, double dt_s) {
    if (armor_id < 0 || armor_id >= 4 || !std::isfinite(dt_s) || dt_s < 0.0 || dt_s > 1.0 ||
        !state.xyz.allFinite() || !state.vxyz.allFinite() || !std::isfinite(state.a) ||
        !std::isfinite(state.w) || !std::isfinite(state.r) || !std::isfinite(state.l) ||
        !std::isfinite(state.h) || state.r <= 0.0) {
        return Eigen::Vector3d::Constant(NAN);
    }
    const bool alternate = armor_id == 1 || armor_id == 3;
    const double radius = state.r + (alternate ? state.l : 0.0);
    if (radius <= 0.0)
        return Eigen::Vector3d::Constant(NAN);
    const double angle = state.a + state.w * dt_s + armor_id * M_PI / 2.0;
    const Eigen::Vector3d center = state.xyz + state.vxyz * dt_s;
    return center - radius * Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0) +
           Eigen::Vector3d(0.0, 0.0, alternate ? state.h : 0.0);
}

GimbalCommand aim_at_tracked_armor(const TargetState& state, int armor_id, double dt_s,
                                   const Eigen::Quaterniond& q_gimbal_to_world,
                                   const Eigen::Vector3d& origin_world, double bullet_speed) {
    const Eigen::Vector3d point_world = predict_tracked_armor(state, armor_id, dt_s);
    const double norm = q_gimbal_to_world.norm();
    if (!point_world.allFinite() || !origin_world.allFinite() || !std::isfinite(norm) ||
        std::abs(norm - 1.0) > 1e-3)
        return {0.0, 0.0, false};
    const Eigen::Vector3d point_gimbal =
        q_gimbal_to_world.normalized().conjugate() * (point_world - origin_world);
    return aim_at(point_gimbal, bullet_speed);
}

AimSelection select_aim_armor(const TargetState& state, const Eigen::Vector3d& gimbal_origin_world,
                              double dt_s, int previous_id, double facing_gate_rad,
                              double hysteresis) {
    AimSelection selection;
    if (!gimbal_origin_world.allFinite() || !std::isfinite(dt_s) || dt_s < 0.0 || dt_s > 1.0 ||
        !std::isfinite(state.a) || !std::isfinite(state.w) || !std::isfinite(state.r))
        return selection;
    const double gate_cosine = std::cos(std::clamp(facing_gate_rad, 0.0, M_PI));
    const double margin = std::max(0.0, hysteresis);

    std::array<double, 4> facing{-2.0, -2.0, -2.0, -2.0};
    int best_id = -1;
    for (int id = 0; id < 4; ++id) {
        const Eigen::Vector3d point = predict_tracked_armor(state, id, dt_s);
        if (!point.allFinite())
            continue;
        const Eigen::Vector3d to_gimbal = gimbal_origin_world - point;
        const double distance = to_gimbal.norm();
        if (!std::isfinite(distance) || distance < 1e-6)
            continue;
        // 注意法向的朝向：Target 的约定是 (cos angle, sin angle) 指向"板→车心"，即内法向。
        // 板面"正对云台"的判据必须用外法向：外法向·(板→云台) > 0。
        // 若直接用内法向取最大，会选中被车体挡住的背面那块板（模拟器 A/B 实测确认过）。
        const double angle = state.a + state.w * dt_s + id * M_PI / 2.0;
        const Eigen::Vector3d outward(-std::cos(angle), -std::sin(angle), 0.0);
        facing[static_cast<std::size_t>(id)] = outward.dot(to_gimbal / distance);
        if (facing[static_cast<std::size_t>(id)] < gate_cosine)
            continue;
        if (best_id < 0 ||
            facing[static_cast<std::size_t>(id)] > facing[static_cast<std::size_t>(best_id)])
            best_id = id;
    }
    if (best_id < 0)
        return selection; // 四块板都侧对：明确失败，不退回"第一块板"

    selection.armor_id = best_id;
    selection.facing = facing[static_cast<std::size_t>(best_id)];

    // 滞回：两块板都好打时保留上一块，避免云台在 45° 附近来回切
    if (previous_id >= 0 && previous_id < 4 && previous_id != best_id) {
        const double previous_facing = facing[static_cast<std::size_t>(previous_id)];
        if (previous_facing >= gate_cosine && previous_facing > selection.facing - margin) {
            selection.armor_id = previous_id;
            selection.facing = previous_facing;
            selection.hysteresis_kept = true;
        }
    }
    return selection;
}

} // namespace diesel_auto_aim
